use std::ops::{AddAssign, SubAssign};
use std::iter;
use std::collections::{HashMap, HashSet};
use std::marker::PhantomData;
use either::Either;

use alga::linear::FiniteDimInnerSpace;
use na::{self, Real, DMatrix, DVector, DVectorSlice, DVectorSliceMut, VectorSliceMutN, LU,
         Dynamic, Vector2, Point2, Point3, MatrixN, Unit};
use ncollide::utils::{self, DeterministicState};
#[cfg(feature = "dim3")]
use ncollide::procedural::{self, IndexBuffer};
use ncollide::shape::{DeformationsType, Triangle, Polyline, Segment};
#[cfg(feature = "dim3")]
use ncollide::shape::{TriMesh, TetrahedronPointLocation};
use ncollide::query::PointQueryWithLocation;

use object::{Body, BodyPart, BodyHandle, BodyPartHandle, BodyStatus, ActivationStatus, FiniteElementIndices};
use solver::{IntegrationParameters, ForceDirection};
use math::{Force, Inertia, Velocity, Vector, Point, Isometry, DIM, Dim};
use object::fem_helper;

/// A triangular element of the mass-LengthConstraint surface.
#[derive(Clone)]
pub struct MassConstraintElement<N: Real> {
    handle: Option<BodyPartHandle>,
    indices: FiniteElementIndices,
    phantom: PhantomData<N>,
}

#[derive(Clone)]
struct LengthConstraint<N: Real> {
    nodes: (usize, usize),
    // Should be Unit<Vector<N>>, but can be zero.
    dir: Unit<Vector<N>>,
    length: N,
    rest_length: N,
    stiffness: Option<N>,
    target_vel: N,
    max_force: N,
}

impl<N: Real> LengthConstraint<N> {
    fn from_positions(nodes: (usize, usize), positions: &[N], stiffness: Option<N>) -> Self {
        let p0 = Point::from_slice(&positions[nodes.0..nodes.0 + DIM]);
        let p1 = Point::from_slice(&positions[nodes.1..nodes.1 + DIM]);
        let rest_length = na::distance(&p0, &p1);

        LengthConstraint {
            nodes,
            dir: Unit::new_normalize(p1 - p0),
            length: rest_length,
            rest_length,
            stiffness,
            max_force: N::zero(),
            target_vel: N::zero(),
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
pub struct MassConstraintSystem<N: Real> {
    handle: Option<BodyHandle>,
    constraints: Vec<LengthConstraint<N>>,
    elements: Vec<MassConstraintElement<N>>,
    kinematic_nodes: DVector<bool>,
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
    warmstart_coeff: N,
}


impl<N: Real> MassConstraintSystem<N> {
    /// Creates a new deformable surface following the mass-LengthConstraint model.
    ///
    /// The surface is initialized with a set of links corresponding to each trimesh edges.
    #[cfg(feature = "dim3")]
    pub fn from_trimesh(mesh: &TriMesh<N>, mass: N, stiffness: Option<N>) -> Self {
        let ndofs = mesh.points().len() * DIM;
        let mut constraints = HashMap::with_hasher(DeterministicState::new());
        let mut elements = Vec::with_capacity(mesh.faces().len());
        let mut positions = DVector::zeros(ndofs);

        for (i, pos) in positions.as_mut_slice().chunks_mut(DIM).enumerate() {
            pos.copy_from_slice(mesh.points()[i].coords.as_slice())
        }

        for face in mesh.faces() {
            let idx = face.indices * DIM;
            let elt = MassConstraintElement {
                handle: None,
                indices: FiniteElementIndices::Triangle(idx),
                phantom: PhantomData
            };

            elements.push(elt);

            let _ = constraints.entry(key(idx.x, idx.y)).or_insert_with(|| {
                LengthConstraint::from_positions((idx.x, idx.y), positions.as_slice(), stiffness)
            });
            let _ = constraints.entry(key(idx.y, idx.z)).or_insert_with(|| {
                LengthConstraint::from_positions((idx.y, idx.z), positions.as_slice(), stiffness)
            });
            let _ = constraints.entry(key(idx.z, idx.x)).or_insert_with(|| {
                LengthConstraint::from_positions((idx.z, idx.x), positions.as_slice(), stiffness)
            });
        }

        let node_mass = mass / na::convert((ndofs / DIM) as f64);

        MassConstraintSystem {
            handle: None,
            constraints: constraints.values().cloned().collect(),
            elements,
            kinematic_nodes: DVector::repeat(mesh.points().len(), false),
            positions,
            velocities: DVector::zeros(ndofs),
            accelerations: DVector::zeros(ndofs),
            impulses: DVector::zeros(0),
            companion_id: 0,
            activation: ActivationStatus::new_active(),
            status: BodyStatus::Dynamic,
            mass,
            node_mass,
            inv_node_mass: N::one() / node_mass,
            warmstart_coeff: na::convert(0.5),
        }
    }

    /// Builds a mass-spring system from a polyline.
    pub fn from_polyline(polyline: &Polyline<N>, mass: N, stiffness: Option<N>) -> Self {
        let ndofs = polyline.points().len() * DIM;
        let mut constraints = HashMap::with_hasher(DeterministicState::new());
        let mut elements = Vec::with_capacity(polyline.edges().len());
        let mut positions = DVector::zeros(ndofs);

        for (i, pos) in positions.as_mut_slice().chunks_mut(DIM).enumerate() {
            pos.copy_from_slice(polyline.points()[i].coords.as_slice())
        }

        for edge in polyline.edges() {
            let idx = edge.indices * DIM;
            let elt = MassConstraintElement {
                handle: None,
                indices: FiniteElementIndices::Segment(idx),
                phantom: PhantomData
            };

            elements.push(elt);

            let _ = constraints.entry(key(idx.x, idx.y)).or_insert_with(|| {
                LengthConstraint::from_positions((idx.x, idx.y), positions.as_slice(), stiffness)
            });
        }

        let node_mass = mass / na::convert((ndofs / DIM) as f64);
        println!("Number of nodes: {}, of constraints: {}", positions.len() / DIM, constraints.len());

        MassConstraintSystem {
            handle: None,
            constraints: constraints.values().cloned().collect(),
            elements,
            kinematic_nodes: DVector::repeat(polyline.points().len(), false),
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
            warmstart_coeff: na::convert(0.5),
        }
    }

    /// Creates a rectangular-shaped quad.
    #[cfg(feature = "dim3")]
    pub fn quad(transform: &Isometry<N>, extents: &Vector2<N>, nx: usize, ny: usize, mass: N, stiffness: Option<N>) -> Self {
        let mesh = procedural::quad(extents.x, extents.y, nx, ny);
        let vertices = mesh.coords.iter().map(|pt| transform * pt).collect();
        let indices = mesh.indices.unwrap_unified().into_iter().map(|tri| na::convert(tri)).collect();
        let trimesh = TriMesh::new(vertices, indices, None);
        Self::from_trimesh(&trimesh, mass, stiffness)
    }

    /// Add one constraint to this mass-constraint system.
    pub fn add_constraint(&mut self, node1: usize, node2: usize, stiffness: Option<N>) {
        assert!(node1 < self.positions.len() / DIM, "Node index out of bounds.");
        assert!(node2 < self.positions.len() / DIM, "Node index out of bounds.");
        let key = key(node1 * DIM, node2 * DIM);
        let constraint = LengthConstraint::from_positions(key, self.positions.as_slice(), stiffness);
        self.constraints.push(constraint);
    }

    /// Generate additional constraints between nodes that are transitively neighbors.
    ///
    /// Given three nodes `a, b, c`, if a constraint exists between `a` and `b`, and between `b` and `c`,
    /// then a constraint between `a` and `c` is created if it does not already exists.
    pub fn generate_neighbor_constraints(&mut self, stiffness: Option<N>) {
        // XXX: duplicate code with MassSpringSurface::generate_neighbor_springs.
        let mut neighbor_list: Vec<_> = iter::repeat(Vec::new()).take(self.positions.len() / DIM).collect();
        let mut existing_constraints = HashSet::with_hasher(DeterministicState::new());

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
                            LengthConstraint::from_positions(key, self.positions.as_slice(), stiffness);
                        self.constraints.push(constraint);
                    }
                }
            }
        }
    }

    /// The coefficient used for warm-starting the resolution of internal constraints of this
    /// soft body (default: 0.5).
    pub fn set_warmstart_coefficient(&mut self, coeff: N) {
        self.warmstart_coeff = coeff
    }

    /// The coefficient used for warm-starting the resolution of internal constraints of this
    /// soft body (default: 0.5).
    pub fn warmstart_coefficient(&self) -> N {
        self.warmstart_coeff
    }
}

impl<N: Real> Body<N> for MassConstraintSystem<N> {
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

        for constraint in &mut self.constraints {
            if let Some(stiffness) = constraint.stiffness {
                let v0 = self.velocities.fixed_rows::<Dim>(constraint.nodes.0);
                let v1 = self.velocities.fixed_rows::<Dim>(constraint.nodes.1);

                let ldot = v1 - v0;
                let l = *constraint.dir;

                // Explicit elastic term.
                let err = constraint.length - constraint.rest_length;
                // FIXME: multiply by erp here too?
                constraint.max_force = stiffness * err.abs();
                constraint.target_vel = params.erp * err / params.dt;

                if err.abs() < params.allowed_linear_error {
                    constraint.max_force = N::zero();
                }
            }
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

    fn fill_constraint_geometry(
        &self,
        part: &BodyPart<N>,
        ndofs: usize, // FIXME: keep this parameter?
        center: &Point<N>,
        force_dir: &ForceDirection<N>,
        j_id: usize,
        wj_id: usize,
        jacobians: &mut [N],
        inv_r: &mut N,
        ext_vels: Option<&DVectorSlice<N>>,
        out_vel: Option<&mut N>
    ) {
        let elt = part.downcast_ref::<MassConstraintElement<N>>().expect("The provided body part must be a triangular mass-spring element");
        fem_helper::fill_contact_geometry_fem(
            self.ndofs(),
            self.status,
            elt.indices,
            &self.positions,
            &self.velocities,
            &self.kinematic_nodes,
            Either::Left(self.inv_node_mass),
            center,
            force_dir,
            j_id,
            wj_id,
            jacobians,
            inv_r,
            ext_vels,
            out_vel
        );
    }

    #[inline]
    fn has_active_internal_constraints(&mut self) -> bool {
        true
    }

    #[inline]
    fn setup_internal_velocity_constraints(&mut self, dvels: &mut DVectorSliceMut<N>) {
        if self.impulses.len() != self.constraints.len() {
            self.impulses = DVector::zeros(self.constraints.len());
        }

        for (i, constraint) in self.constraints.iter().enumerate() {
            if constraint.stiffness.is_some() {
                self.impulses[i] *= self.warmstart_coeff;
            }

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
        for (i, constraint) in self.constraints.iter_mut().enumerate() {
            let v0 = self.velocities.fixed_rows::<Dim>(constraint.nodes.0) + dvels.fixed_rows::<Dim>(constraint.nodes.0);
            let v1 = self.velocities.fixed_rows::<Dim>(constraint.nodes.1) + dvels.fixed_rows::<Dim>(constraint.nodes.1);

            let dvel = (v1 - v0).dot(&constraint.dir);
            let dlambda;
            if let Some(stiffness) = constraint.stiffness {
                let curr_impulse = self.impulses[i];
                let dimpulse = (dvel + constraint.target_vel) / (self.inv_node_mass + self.inv_node_mass);
                let new_impulse = na::clamp(curr_impulse + dimpulse, -constraint.max_force, constraint.max_force);
                dlambda = new_impulse - curr_impulse;
                self.impulses[i] = new_impulse;
            } else {
                dlambda = dvel / (self.inv_node_mass + self.inv_node_mass);
                self.impulses[i] += dlambda;
            }

            let vel_correction = *constraint.dir * (dlambda * self.inv_node_mass);
            dvels.fixed_rows_mut::<Dim>(constraint.nodes.0).add_assign(&vel_correction);
            dvels.fixed_rows_mut::<Dim>(constraint.nodes.1).sub_assign(&vel_correction);
        }
    }

    #[inline]
    fn step_solve_internal_position_constraints(&mut self, params: &IntegrationParameters<N>) {
        for (i, constraint) in self.constraints.iter_mut().enumerate() {
            if constraint.stiffness.is_none() {
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