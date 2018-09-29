use std::ops::{AddAssign, SubAssign};
use std::iter;
use std::collections::{HashMap, HashSet};

use alga::linear::FiniteDimInnerSpace;
use na::{self, Real, DMatrix, DVector, DVectorSlice, DVectorSliceMut, VectorSliceMutN, LU,
         Dynamic, Vector2, Point3, MatrixN, Unit};
use ncollide::utils::{self, DeterministicState};
use ncollide::procedural::{self, IndexBuffer};
use ncollide::shape::{TriMesh, DeformationsType, TetrahedronPointLocation, Triangle};
use ncollide::query::PointQueryWithLocation;

use object::{Body, BodyPart, BodyHandle, BodyPartHandle, BodyStatus, ActivationStatus};
use solver::{IntegrationParameters, ForceDirection};
use math::{Force, Inertia, Velocity, Vector, Point, Isometry, DIM, Dim};

/// A triangular element of the mass-LengthConstraint surface.
#[derive(Clone)]
pub struct MassConstraintElement<N: Real> {
    handle: Option<BodyPartHandle>,
    indices: Point3<usize>,
    surface: N,
}

#[derive(Clone)]
struct LengthConstraint<N: Real> {
    nodes: (usize, usize),
    // Should be Unit<Vector<N>>, but can be zero.
    dir: Unit<Vector<N>>,
    length: N,
    rest_length: N,
    max_anti_compression_force: N,
    max_anti_extension_force: N,
}

impl<N: Real> LengthConstraint<N> {
    fn from_positions(nodes: (usize, usize), positions: &[N], max_anti_compression_force: Option<N>, max_anti_extension_force: Option<N>) -> Self {
        let p0 = Point::from_slice(&positions[nodes.0..nodes.0 + DIM]);
        let p1 = Point::from_slice(&positions[nodes.1..nodes.1 + DIM]);
        let rest_length = na::distance(&p0, &p1);

        LengthConstraint {
            nodes,
            dir: Unit::new_normalize(p1 - p0),
            length: rest_length,
            rest_length,
            max_anti_compression_force: max_anti_compression_force.unwrap_or(N::max_value()),
            max_anti_extension_force: max_anti_extension_force.unwrap_or(N::max_value()),
        }
    }
}

fn key(i: usize, j: usize) -> (usize, usize) {
    if i <= j {
        (i, j)
    } else {
        (j, i)
    }
}

/// A deformable surface using a mass-LengthConstraint model with triangular elements.
#[derive(Clone)]
pub struct MassConstraintSurface<N: Real> {
    handle: Option<BodyHandle>,
    constraints: Vec<LengthConstraint<N>>,
    elements: Vec<MassConstraintElement<N>>,
    positions: DVector<N>,
    velocities: DVector<N>,
    accelerations: DVector<N>,
    impulses: DVector<N>,

    companion_id: usize,
    activation: ActivationStatus<N>,
    status: BodyStatus,
    mass: N,
    node_mass: N,
    inv_node_mass: N,
}


impl<N: Real> MassConstraintSurface<N> {
    /// Creates a new deformable surface following the mass-LengthConstraint model.
    ///
    /// The surface is initialized with a set of links corresponding to each trimesh edges.
    pub fn new(mesh: &TriMesh<N>, mass: N, max_anti_compression_force: Option<N>, max_anti_extension_force: Option<N>) -> Self {
        let ndofs = mesh.vertices().len() * DIM;
        let mut constraints = HashMap::with_hasher(DeterministicState::new());
        let mut elements = Vec::with_capacity(mesh.indices().len());
        let mut positions = DVector::zeros(ndofs);

        for (i, pos) in positions.as_mut_slice().chunks_mut(DIM).enumerate() {
            pos.copy_from_slice(mesh.vertices()[i].coords.as_slice())
        }

        for idx in mesh.indices() {
            let idx = idx * DIM;
            let elt = MassConstraintElement {
                handle: None,
                indices: idx,
                surface: N::zero(),
            };

            elements.push(elt);

            let _ = constraints.entry(key(idx.x, idx.y)).or_insert_with(|| {
                LengthConstraint::from_positions((idx.x, idx.y), positions.as_slice(), max_anti_compression_force, max_anti_extension_force)
            });
            let _ = constraints.entry(key(idx.y, idx.z)).or_insert_with(|| {
                LengthConstraint::from_positions((idx.y, idx.z), positions.as_slice(), max_anti_compression_force, max_anti_extension_force)
            });
            let _ = constraints.entry(key(idx.z, idx.x)).or_insert_with(|| {
                LengthConstraint::from_positions((idx.z, idx.x), positions.as_slice(), max_anti_compression_force, max_anti_extension_force)
            });
        }

        let node_mass = mass / na::convert((ndofs / DIM) as f64);

        MassConstraintSurface {
            handle: None,
            constraints: constraints.values().cloned().collect(),
            elements,
            positions,
            velocities: DVector::zeros(ndofs),
            accelerations: DVector::zeros(ndofs),
            impulses: DVector::zeros(constraints.len()),
            companion_id: 0,
            activation: ActivationStatus::new_active(),
            status: BodyStatus::Dynamic,
            mass,
            node_mass,
            inv_node_mass: N::one() / node_mass,
        }
    }

    /// Creates a rectangular-shaped quad.
    pub fn quad(transform: &Isometry<N>, extents: &Vector2<N>, nx: usize, ny: usize, mass: N, max_anti_compression_force: Option<N>, max_anti_extension_force: Option<N>) -> Self {
        let mesh = procedural::quad(extents.x, extents.y, nx, ny);
        let vertices = mesh.coords.iter().map(|pt| transform * pt).collect();
        let indices = mesh.indices.unwrap_unified().into_iter().map(|tri| na::convert(tri)).collect();
        let trimesh = TriMesh::new(vertices, indices, None);
        Self::new(&trimesh, mass, max_anti_compression_force, max_anti_extension_force)
    }

    /// The triangle mesh corresponding to this mass-LengthConstraint-surface structural elements.
    pub fn mesh(&self) -> TriMesh<N> {
        let vertices = self.positions.as_slice().chunks(DIM).map(|pt| Point::from_coordinates(Vector::from_row_slice(pt))).collect();
        let indices = self.elements.iter().map(|elt| elt.indices / DIM).collect();

        TriMesh::new(vertices, indices, None)
    }

    /// Generate additional constraints between nodes that are transitively neighbors.
    ///
    /// Given three nodes `a, b, c`, if a constraint exists between `a` and `b`, and between `b` and `c`,
    /// then a constraint between `a` and `c` is created if it does not already exists.
    pub fn generate_neighbor_constraints(&mut self, max_anti_compression_force: Option<N>, max_anti_extension_force: Option<N>) {
        // XXX: duplicate code with MassSpringSurface::generate_neighbor_springs.
        let mut neighbor_list: Vec<_> = iter::repeat(Vec::new()).take(self.positions.len() / 3).collect();
        let mut existing_constraints = HashSet::new();

        // Build neighborhood list.
        for constraint in &self.constraints {
            let key = key(constraint.nodes.0, constraint.nodes.1);
            neighbor_list[key.0 / DIM].push(key.1 / DIM);
            let _ = existing_constraints.insert(key);
        }

        // Build constraints.
        for (i, nbhs) in neighbor_list.iter().enumerate() {
            for nbh in nbhs {
                for transitive_nbh in &neighbor_list[*nbh] {
                    let key = key(i * DIM, *transitive_nbh * DIM);

                    if existing_constraints.insert(key) {
                        let constraint =
                            LengthConstraint::from_positions(key, self.positions.as_slice(), max_anti_compression_force, max_anti_extension_force);
                        self.constraints.push(constraint);
                    }
                }
            }
        }

        self.impulses = DVector::zeros(self.constraints.len());
    }
}

impl<N: Real> Body<N> for MassConstraintSurface<N> {
    fn update_kinematics(&mut self) {
        for constraint in &mut self.constraints {
            let p0 = self.positions.fixed_rows::<Dim>(constraint.nodes.0);
            let p1 = self.positions.fixed_rows::<Dim>(constraint.nodes.1);
            let l = p1 - p0;

            if let Some((dir, length)) = Unit::try_new_and_get(l, N::zero()) {
                constraint.dir = dir;
                constraint.length = length;
            } else {
                constraint.dir = Vector::y_axis();
                constraint.length = N::zero();
            }
        }
    }

    fn update_dynamics(&mut self, gravity: &Vector<N>, params: &IntegrationParameters<N>) {
        let gravity_acc = gravity;

        for i in 0..self.positions.len() / DIM {
            let mut acc = self.accelerations.fixed_rows_mut::<Dim>(i * DIM);
            acc += gravity_acc
        }
    }

    fn clear_dynamics(&mut self) {
        self.accelerations.fill(N::zero());
    }

    fn apply_displacement(&mut self, disp: &[N]) {
        let disp = DVectorSlice::from_slice(disp, self.positions.len());
        self.positions += disp;
    }

    fn set_handle(&mut self, handle: Option<BodyHandle>) {
        self.handle = handle;

        for (i, element) in self.elements.iter_mut().enumerate() {
            element.handle = handle.map(|h| BodyPartHandle { body_handle: h, part_id: i })
        }
    }

    fn handle(&self) -> Option<BodyHandle> {
        self.handle
    }

    fn status(&self) -> BodyStatus {
        self.status
    }

    fn set_status(&mut self, status: BodyStatus) {
        self.status = status
    }

    fn activation_status(&self) -> &ActivationStatus<N> {
        &self.activation
    }

    fn set_deactivation_threshold(&mut self, threshold: Option<N>) {
        self.activation.set_deactivation_threshold(threshold)
    }

    fn ndofs(&self) -> usize {
        self.positions.len()
    }

    fn generalized_acceleration(&self) -> DVectorSlice<N> {
        DVectorSlice::from_slice(self.accelerations.as_slice(), self.accelerations.len())
    }

    fn generalized_velocity(&self) -> DVectorSlice<N> {
        DVectorSlice::from_slice(self.velocities.as_slice(), self.velocities.len())
    }

    fn companion_id(&self) -> usize {
        self.companion_id
    }

    fn set_companion_id(&mut self, id: usize) {
        self.companion_id = id
    }

    fn generalized_velocity_mut(&mut self) -> DVectorSliceMut<N> {
        let len = self.velocities.len();
        DVectorSliceMut::from_slice(self.velocities.as_mut_slice(), len)
    }

    fn integrate(&mut self, params: &IntegrationParameters<N>) {
        self.positions.axpy(params.dt, &self.velocities, N::one())
    }

    fn activate_with_energy(&mut self, energy: N) {
        self.activation.set_energy(energy)
    }

    fn deactivate(&mut self) {
        self.activation.set_energy(N::zero());
        self.velocities.fill(N::zero());
    }

    fn part(&self, handle: BodyPartHandle) -> &BodyPart<N> {
        &self.elements[handle.part_id]
    }

    fn part_mut(&mut self, handle: BodyPartHandle) -> &mut BodyPart<N> {
        &mut self.elements[handle.part_id]
    }

    fn contains_part(&self, handle: BodyPartHandle) -> bool {
        if let Some(me) = self.handle {
            handle.body_handle == me && handle.part_id < self.elements.len()
        } else {
            false
        }
    }

    fn deformed_positions(&self) -> Option<(DeformationsType, &[N])> {
        Some((DeformationsType::Vectors, self.positions.as_slice()))
    }

    fn deformed_positions_mut(&mut self) -> Option<(DeformationsType, &mut [N])> {
        Some((DeformationsType::Vectors, self.positions.as_mut_slice()))
    }

    fn inv_mass_mul_generalized_forces(&self, generalized_forces: &mut [N]) {
        let mut out = DVectorSliceMut::from_slice(generalized_forces, self.ndofs());
        out /= self.inv_node_mass;
    }

    fn body_part_jacobian_mul_unit_force(&self, part: &BodyPart<N>, pt: &Point<N>, force_dir: &ForceDirection<N>, out: &mut [N]) {
        // Needed by the non-linear SOR-prox.
        // FIXME: should this be done by the non-linear SOR-prox itself?
        DVectorSliceMut::from_slice(out, self.ndofs()).fill(N::zero());

        if let ForceDirection::Linear(dir) = force_dir {
            let elt = part.downcast_ref::<MassConstraintElement<N>>().expect("The provided body part must be a triangular mass-LengthConstraint element");

            let a = self.positions.fixed_rows::<Dim>(elt.indices.x).into_owned();
            let b = self.positions.fixed_rows::<Dim>(elt.indices.y).into_owned();
            let c = self.positions.fixed_rows::<Dim>(elt.indices.z).into_owned();

            let tri = Triangle::new(
                Point::from_coordinates(a),
                Point::from_coordinates(b),
                Point::from_coordinates(c),
            );

            // XXX: This is extremely costly!
            let proj = tri.project_point_with_location(&Isometry::identity(), pt, false).1;
            let bcoords = proj.barycentric_coordinates().unwrap();

            VectorSliceMutN::<N, Dim>::from_slice(&mut out[elt.indices.x..]).copy_from(&(**dir * bcoords[0]));
            VectorSliceMutN::<N, Dim>::from_slice(&mut out[elt.indices.y..]).copy_from(&(**dir * bcoords[1]));
            VectorSliceMutN::<N, Dim>::from_slice(&mut out[elt.indices.z..]).copy_from(&(**dir * bcoords[2]));
        }
    }

    fn body_part_point_velocity(&self, part: &BodyPart<N>, point: &Point<N>, force_dir: &ForceDirection<N>) -> N {
        unimplemented!()
    }

    #[inline]
    fn has_active_internal_constraints(&mut self) -> bool {
        true
    }

    #[inline]
    fn setup_internal_velocity_constraints(&mut self, dvels: &mut DVectorSliceMut<N>) {
        for (i, constraint) in self.constraints.iter().enumerate() {
            let impulse = self.impulses[i];
            if !impulse.is_zero() {
                let vel_correction = *constraint.dir * (impulse * self.inv_node_mass);
                dvels.fixed_rows_mut::<Dim>(constraint.nodes.0).add_assign(&vel_correction);
                dvels.fixed_rows_mut::<Dim>(constraint.nodes.1).sub_assign(&vel_correction);
            }
        }
    }

    #[inline]
    fn step_solve_internal_velocity_constraints(&mut self, dvels: &mut DVectorSliceMut<N>) {
        // Solve internal constraints using a PGS solver.
        // Note that we use the mass matrix (instead of the augmented mass
        // matrix) for solving those constraints.
        // The mass matrix is simply a diagonal with elements equal to self.mass / self.
        for (i, constraint) in self.constraints.iter().enumerate() {
            let v0 = self.velocities.fixed_rows::<Dim>(constraint.nodes.0) + dvels.fixed_rows::<Dim>(constraint.nodes.0);
            let v1 = self.velocities.fixed_rows::<Dim>(constraint.nodes.1) + dvels.fixed_rows::<Dim>(constraint.nodes.1);

            let dvel = (v1 - v0).dot(&constraint.dir);

            let curr_impulse = self.impulses[i];
            let new_impulse = na::clamp(
                curr_impulse + dvel / (self.inv_node_mass + self.inv_node_mass),
                -constraint.max_anti_compression_force,
                constraint.max_anti_extension_force,
            );
            let dlambda = new_impulse - curr_impulse;
            self.impulses[i] = new_impulse;

            let vel_correction = *constraint.dir * (dlambda * self.inv_node_mass);
            dvels.fixed_rows_mut::<Dim>(constraint.nodes.0).add_assign(&vel_correction);
            dvels.fixed_rows_mut::<Dim>(constraint.nodes.1).sub_assign(&vel_correction);
        }
    }

    #[inline]
    fn step_solve_internal_position_constraints(&mut self, params: &IntegrationParameters<N>) {
        for (i, constraint) in self.constraints.iter_mut().enumerate() {
            if self.impulses[i] == -constraint.max_anti_compression_force || self.impulses[i] == constraint.max_anti_extension_force {
                // Don't apply stabilization if the constraint forces reached their max.
                continue;
            }

            let dpos = self.positions.fixed_rows::<Dim>(constraint.nodes.1)
                - self.positions.fixed_rows::<Dim>(constraint.nodes.0);

            if let Some((dir, length)) = Unit::try_new_and_get(dpos, N::zero()) {
                constraint.dir = dir;
                constraint.length = length;
            }

            let error = constraint.length - constraint.rest_length;
            let clamped_error = if error > N::zero() {
                na::clamp(
                    (error - params.allowed_linear_error) * params.erp,
                    N::zero(),
                    params.max_linear_correction,
                )
            } else {
                na::clamp(
                    (error + params.allowed_linear_error) * params.erp,
                    -params.max_linear_correction,
                    N::zero(),
                )
            };

            if !clamped_error.is_zero() {
                let shift = *constraint.dir * (clamped_error * na::convert(0.5));
                self.positions.fixed_rows_mut::<Dim>(constraint.nodes.0).add_assign(&shift);
                self.positions.fixed_rows_mut::<Dim>(constraint.nodes.1).sub_assign(&shift);
            }
        }
    }
}


impl<N: Real> BodyPart<N> for MassConstraintElement<N> {
    fn handle(&self) -> Option<BodyPartHandle> {
        self.handle
    }

    fn center_of_mass(&self) -> Point<N> {
        unimplemented!()
    }

    fn position(&self) -> Isometry<N> {
        Isometry::new(Vector::<N>::y() * na::convert::<_, N>(100.0f64), na::zero()) // XXX
    }

    fn velocity(&self) -> Velocity<N> {
        unimplemented!()
    }

    fn inertia(&self) -> Inertia<N> {
        unimplemented!()
    }

    fn local_inertia(&self) -> Inertia<N> {
        unimplemented!()
    }

    fn apply_force(&mut self, force: &Force<N>) {
        unimplemented!()
    }
}