use std::ops::{AddAssign, SubAssign};
use std::iter;
use std::collections::HashMap;

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

/// A triangular element of the mass-spring surface.
#[derive(Clone)]
pub struct MassSpringElement<N: Real> {
    handle: Option<BodyPartHandle>,
    indices: Point3<usize>,
    surface: N,
}

#[derive(Copy, Clone, PartialEq, Eq)]
enum SpringConstraintType<N: Real> {
    Equal,
    Min(N),
    Max(N),
}

#[derive(Clone)]
struct Spring<N: Real> {
    nodes: (usize, usize),
    // Should be Unit<Vector<N>>, but can be zero.
    dir: Unit<Vector<N>>,
    length: N,
    rest_length: N,
}

/// The type of a single spring for a deformable body based on the mass-spring model.
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum SpringType<N> {
    /// Spring with an infinite spring coefficient, i.e., with a constant length.
    Rigid,
    /// A spring with a non-constant length.
    Elastic {
        /// The spring coefficient.
        stiffness: N,
        /// The spring damping coefficient.
        damping: N,
        /// The minimum spring length.
        min_length: Option<N>,
        /// The maximum spring length.
        max_length: Option<N>,
    },
}

impl<N: Real> Spring<N> {
    fn from_positions(nodes: (usize, usize), positions: &[N]) -> Self {
        let p0 = Point::from_slice(&positions[nodes.0..nodes.0 + DIM]);
        let p1 = Point::from_slice(&positions[nodes.1..nodes.1 + DIM]);
        let rest_length = na::distance(&p0, &p1);

        Spring {
            nodes,
            dir: Unit::new_normalize(p1 - p0),
            length: rest_length,
            rest_length,
        }
    }
}

/// A deformable surface using a mass-spring model with triangular elements.
#[derive(Clone)]
pub struct MassSpringSurface<N: Real> {
    handle: Option<BodyHandle>,
    springs: Vec<Spring<N>>,
    elements: Vec<MassSpringElement<N>>,
    positions: DVector<N>,
    velocities: DVector<N>,
    accelerations: DVector<N>,
    augmented_mass: DMatrix<N>,
    inv_augmented_mass: LU<N, Dynamic, Dynamic>,
    active_spring_constraints: Vec<(usize, SpringConstraintType<N>)>,
    impulses: DVector<N>,
    impulse_limits: Vec<(N, N)>,

    companion_id: usize,
    activation: ActivationStatus<N>,
    status: BodyStatus,
    springs_type: SpringType<N>,
    mass: N,
    node_mass: N,
    inv_node_mass: N,
}

impl<N: Real> MassSpringSurface<N> {
    /// Creates a new deformable surface following the mass-spring model.
    ///
    /// The surface is initialized with a set of links corresponding to each trimesh edges.
    pub fn new(mesh: &TriMesh<N>, mass: N, springs_type: SpringType<N>) -> Self {
        let ndofs = mesh.vertices().len() * DIM;
        let mut springs = HashMap::with_hasher(DeterministicState::new());
        let mut elements = Vec::with_capacity(mesh.indices().len());
        let mut positions = DVector::zeros(ndofs);

        for (i, pos) in positions.as_mut_slice().chunks_mut(DIM).enumerate() {
            pos.copy_from_slice(mesh.vertices()[i].coords.as_slice())
        }

        for idx in mesh.indices() {
            let idx = idx * DIM;
            let elt = MassSpringElement {
                handle: None,
                indices: idx,
                surface: N::zero(),
            };

            elements.push(elt);

            fn key(i: usize, j: usize) -> (usize, usize) {
                if i <= j {
                    (i, j)
                } else {
                    (j, i)
                }
            }

            let _ = springs.entry(key(idx.x, idx.y)).or_insert_with(|| {
                Spring::from_positions((idx.x, idx.y), positions.as_slice())
            });
            let _ = springs.entry(key(idx.y, idx.z)).or_insert_with(|| {
                Spring::from_positions((idx.y, idx.z), positions.as_slice())
            });
            let _ = springs.entry(key(idx.z, idx.x)).or_insert_with(|| {
                Spring::from_positions((idx.z, idx.x), positions.as_slice())
            });
        }

        let has_constraints = match springs_type {
            SpringType::Rigid => true,
            SpringType::Elastic { min_length, max_length, .. } => min_length.is_some() || max_length.is_some(),
        };

        let impulses;
        if has_constraints {
            impulses = DVector::zeros(springs.len());
        } else {
            impulses = DVector::zeros(0);
        }

        let node_mass = mass / na::convert((ndofs / DIM) as f64);

        MassSpringSurface {
            handle: None,
            springs: springs.values().cloned().collect(),
            elements,
            positions,
            velocities: DVector::zeros(ndofs),
            accelerations: DVector::zeros(ndofs),
            augmented_mass: DMatrix::zeros(ndofs, ndofs),
            inv_augmented_mass: LU::new(DMatrix::zeros(0, 0)),
            active_spring_constraints: Vec::new(),
            impulses,
            impulse_limits: Vec::new(),
            companion_id: 0,
            activation: ActivationStatus::new_active(),
            status: BodyStatus::Dynamic,
            springs_type,
            mass,
            node_mass,
            inv_node_mass: N::one() / node_mass,
        }
    }

    /// Creates a rectangular-shaped quad.
    pub fn quad(transform: &Isometry<N>, extents: &Vector2<N>, nx: usize, ny: usize, mass: N, springs_type: SpringType<N>) -> Self {
        let mesh = procedural::quad(extents.x, extents.y, nx, ny);
        let vertices = mesh.coords.iter().map(|pt| transform * pt).collect();
        let indices = mesh.indices.unwrap_unified().into_iter().map(|tri| na::convert(tri)).collect();
        let trimesh = TriMesh::new(vertices, indices, None);
        Self::new(&trimesh, mass, springs_type)
    }

    /// The triangle mesh corresponding to this mass-spring-surface structural elements.
    pub fn mesh(&self) -> TriMesh<N> {
        let vertices = self.positions.as_slice().chunks(DIM).map(|pt| Point::from_coordinates(Vector::from_row_slice(pt))).collect();
        let indices = self.elements.iter().map(|elt| elt.indices / DIM).collect();

        TriMesh::new(vertices, indices, None)
    }

    fn update_gravity_force(&mut self, gravity: &Vector<N>) {
        let gravity_force = gravity * self.node_mass;

        for i in 0..self.positions.len() / DIM {
            let mut acc = self.accelerations.fixed_rows_mut::<Dim>(i * DIM);
            acc += gravity_force
        }
    }

    fn update_active_constraints_list(&mut self) {
        match self.springs_type {
            SpringType::Rigid => {
                // FIXME: don't do this, find another way
                // to say that all spring constraints are active.
                for i in 0..self.springs.len() {
                    self.active_spring_constraints.push((i, SpringConstraintType::Equal));
                }
            }
            SpringType::Elastic { min_length, max_length, .. } => {
                if min_length.is_none() && max_length.is_none() {
                    return;
                }

                let min = min_length.unwrap_or(N::min_value());
                let max = max_length.unwrap_or(N::max_value());
                for (i, spring) in self.springs.iter().enumerate() {
                    if spring.length <= min {
                        self.active_spring_constraints.push((i, SpringConstraintType::Min(min)))
                    } else if spring.length >= max {
                        self.active_spring_constraints.push((i, SpringConstraintType::Max(max)))
                    }
                }
            }
        }
    }

    fn update_augmented_mass_and_forces(&mut self, gravity: &Vector<N>, params: &IntegrationParameters<N>) {
        self.augmented_mass.fill_diagonal(self.node_mass);

        if let SpringType::Elastic { stiffness, damping, .. } = self.springs_type {
            for spring in &mut self.springs {
                let v0 = self.velocities.fixed_rows::<Dim>(spring.nodes.0);
                let v1 = self.velocities.fixed_rows::<Dim>(spring.nodes.1);

                let ldot = v1 - v0;
                let l = *spring.dir;

                // Explicit elastic term.
                let coeff = stiffness * (spring.length - spring.rest_length) + damping * ldot.dot(&l);
                let f0 = l * coeff;

                self.accelerations.fixed_rows_mut::<Dim>(spring.nodes.0).add_assign(&f0);
                self.accelerations.fixed_rows_mut::<Dim>(spring.nodes.1).sub_assign(&f0);

                if spring.length != N::zero() {
                    /*
                     *
                     * Stiffness matrix contribution: there are 4 terms.
                     *
                     */
                    // let contrib0 = MatrixN::<N, Dim>::from_diagonal_element(coeff / length);
                    // let contrib1 = l * (l.transpose() * (stiffness * spring.rest_length / length));
                    // let contrib2 = l * (ldot.transpose() * (damping / length));
                    // let contrib3 = -l * (l.transpose() * (damping / length * ldot.dot(&l) * na::convert(2.0)));
                    // let contrib = contrib0 + contrib1 + contrib2 + contrib3;
                    // More compact version bellow:
                    let contrib0 = MatrixN::<N, Dim>::from_diagonal_element(coeff / spring.length);
                    let contrib1 = ldot * (damping / spring.length);
                    let contrib23 = l * ((stiffness * spring.rest_length - damping * ldot.dot(&l) * na::convert(2.0)) / spring.length);
                    let stiffness = contrib0 + l * (contrib1 + contrib23).transpose();

                    let forward_f0 = stiffness * (ldot * params.dt);

                    // Add the contributions to the forces.
                    self.accelerations.fixed_rows_mut::<Dim>(spring.nodes.0).add_assign(&forward_f0);
                    self.accelerations.fixed_rows_mut::<Dim>(spring.nodes.1).sub_assign(&forward_f0);

                    // Damping matrix contribution.
                    let damping_dt = (l * l.transpose()) * (damping * params.dt);

                    // Add to the mass matrix.
                    let damping_stiffness = damping_dt - stiffness * (params.dt * params.dt);
                    self.augmented_mass.fixed_slice_mut::<Dim, Dim>(spring.nodes.0, spring.nodes.0).add_assign(&damping_stiffness);
                    self.augmented_mass.fixed_slice_mut::<Dim, Dim>(spring.nodes.1, spring.nodes.1).add_assign(&damping_stiffness);
                    self.augmented_mass.fixed_slice_mut::<Dim, Dim>(spring.nodes.0, spring.nodes.1).sub_assign(&damping_stiffness);
                    self.augmented_mass.fixed_slice_mut::<Dim, Dim>(spring.nodes.1, spring.nodes.0).sub_assign(&damping_stiffness);
                }
            }
        }

        self.update_gravity_force(gravity);
        self.inv_augmented_mass = LU::new(self.augmented_mass.clone());
        assert!(self.inv_augmented_mass.solve_mut(&mut self.accelerations));
    }
}

impl<N: Real> Body<N> for MassSpringSurface<N> {
    fn update_kinematics(&mut self) {
        for spring in &mut self.springs {
            let p0 = self.positions.fixed_rows::<Dim>(spring.nodes.0);
            let p1 = self.positions.fixed_rows::<Dim>(spring.nodes.1);
            let l = p1 - p0;

            if let Some((dir, length)) = Unit::try_new_and_get(l, N::zero()) {
                spring.dir = dir;
                spring.length = length;
            } else {
                spring.dir = Vector::y_axis();
                spring.length = N::zero();
            }
        }
    }

    fn update_dynamics(&mut self, gravity: &Vector<N>, params: &IntegrationParameters<N>) {
        self.update_augmented_mass_and_forces(gravity, params);
        self.update_active_constraints_list();
    }

    fn clear_dynamics(&mut self) {
        self.accelerations.fill(N::zero());
        self.augmented_mass.fill(N::zero());
        self.active_spring_constraints.clear();
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
        assert!(self.inv_augmented_mass.solve_mut(&mut out))
    }

    fn body_part_jacobian_mul_unit_force(&self, part: &BodyPart<N>, pt: &Point<N>, force_dir: &ForceDirection<N>, out: &mut [N]) {
        // Needed by the non-linear SOR-prox.
        // FIXME: should this be done by the non-linear SOR-prox itself?
        DVectorSliceMut::from_slice(out, self.ndofs()).fill(N::zero());

        if let ForceDirection::Linear(dir) = force_dir {
            let elt = part.downcast_ref::<MassSpringElement<N>>().expect("The provided body part must be a triangular mass-spring element");

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
        !self.active_spring_constraints.is_empty()
    }

    #[inline]
    fn setup_internal_velocity_constraints(&mut self, dvels: &mut DVectorSliceMut<N>) {
        for i in &self.active_spring_constraints {
            let impulse = self.impulses[i.0];
            if !impulse.is_zero() {
                let spring = &self.springs[i.0];
                let vel_correction = *spring.dir * (impulse * self.inv_node_mass);
                dvels.fixed_rows_mut::<Dim>(spring.nodes.0).add_assign(&vel_correction);
                dvels.fixed_rows_mut::<Dim>(spring.nodes.1).sub_assign(&vel_correction);
            }
        }
    }

    #[inline]
    fn step_solve_internal_velocity_constraints(&mut self, dvels: &mut DVectorSliceMut<N>) {
        // Solve internal constraints using a PGS solver.
        // Note that we use the mass matrix (instead of the augmented mass
        // matrix) for solving those constraints.
        // The mass matrix is simply a diagonal with elements equal to self.mass / self.
        for c in &self.active_spring_constraints {
            let spring = &self.springs[c.0];
            let v0 = self.velocities.fixed_rows::<Dim>(spring.nodes.0) + dvels.fixed_rows::<Dim>(spring.nodes.0);
            let v1 = self.velocities.fixed_rows::<Dim>(spring.nodes.1) + dvels.fixed_rows::<Dim>(spring.nodes.1);

            let dvel = (v1 - v0).dot(&spring.dir);
            let curr_impulse = self.impulses[c.0];
            let mut new_impulse = curr_impulse + dvel / (self.inv_node_mass + self.inv_node_mass);

            match c.1 {
                SpringConstraintType::Min(_) => {
                    new_impulse = na::sup(&N::zero(), &new_impulse);
                }
                SpringConstraintType::Max(_) => {
                    new_impulse = na::inf(&N::zero(), &new_impulse);
                }
                SpringConstraintType::Equal => {}
            };

            let dlambda = new_impulse - curr_impulse;
            self.impulses[c.0] = new_impulse;

            let vel_correction = *spring.dir * (dlambda * self.inv_node_mass);

            dvels.fixed_rows_mut::<Dim>(spring.nodes.0).add_assign(&vel_correction);
            dvels.fixed_rows_mut::<Dim>(spring.nodes.1).sub_assign(&vel_correction);
        }
    }

    #[inline]
    fn step_solve_internal_position_constraints(&mut self, params: &IntegrationParameters<N>) {
        for c in &self.active_spring_constraints {
            let spring = &mut self.springs[c.0];
            let dpos = self.positions.fixed_rows::<Dim>(spring.nodes.1)
                - self.positions.fixed_rows::<Dim>(spring.nodes.0);

            if let Some((dir, length)) = Unit::try_new_and_get(dpos, N::zero()) {
                spring.dir = dir;
                spring.length = length;
            }

            let clamped_error = match c.1 {
                SpringConstraintType::Min(bound) => {
                    na::clamp(
                        (spring.length - bound + params.allowed_linear_error) * params.erp,
                        -params.max_linear_correction,
                        N::zero(),
                    )
                }
                SpringConstraintType::Max(bound) => {
                    na::clamp(
                        (spring.length - bound - params.allowed_linear_error) * params.erp,
                        N::zero(),
                        params.max_linear_correction,
                    )
                }
                SpringConstraintType::Equal => {
                    let error = spring.length - spring.rest_length;

                    if error > N::zero() {
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
                    }
                }
            };


            if !clamped_error.is_zero() {
                let shift = *spring.dir * (clamped_error * na::convert(0.5));
                self.positions.fixed_rows_mut::<Dim>(spring.nodes.0).add_assign(&shift);
                self.positions.fixed_rows_mut::<Dim>(spring.nodes.1).sub_assign(&shift);
            }
        }
    }
}


impl<N: Real> BodyPart<N> for MassSpringElement<N> {
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