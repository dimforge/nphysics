use std::ops::{AddAssign, SubAssign};
use std::iter;
use std::collections::{HashMap, HashSet};
use std::marker::PhantomData;
use std::any::Any;
use either::Either;

use na::{self, RealField, DVector, DVectorSlice, DVectorSliceMut, Unit};
#[cfg(feature = "dim3")]
use na::Vector2;
use ncollide::utils::DeterministicState;
#[cfg(feature = "dim3")]
use ncollide::procedural;
use ncollide::shape::{DeformationsType, Polyline, ShapeHandle};
#[cfg(feature = "dim3")]
use ncollide::shape::TriMesh;

use crate::object::{Body, BodyPart, BodyHandle, BodyPartHandle, BodyStatus, BodyUpdateStatus,
                    ActivationStatus, FiniteElementIndices, DeformableColliderDesc, BodyDesc};
use crate::solver::{IntegrationParameters, ForceDirection};
use crate::math::{Force, ForceType, Inertia, Velocity, Vector, Point, Isometry, DIM, Dim, Translation};
use crate::object::fem_helper;
use crate::world::{World, ColliderWorld};
use crate::utils::{UserData, UserDataBox};

/// A triangular element of the mass-LengthConstraint surface.
#[derive(Clone)]
pub struct MassConstraintElement<N: RealField> {
    handle: BodyPartHandle,
    indices: FiniteElementIndices,
    phantom: PhantomData<N>,
}

#[derive(Clone)]
struct LengthConstraint<N: RealField> {
    nodes: (usize, usize),
    // Should be Unit<Vector<N>>, but can be zero.
    dir: Unit<Vector<N>>,
    length: N,
    rest_length: N,
    stiffness: Option<N>,
    target_vel: N,
    max_force: N,
    plastic_strain: N
}

impl<N: RealField> LengthConstraint<N> {
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
            plastic_strain: N::zero()
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
pub struct MassConstraintSystem<N: RealField> {
    name: String,
    handle: BodyHandle,
    constraints: Vec<LengthConstraint<N>>,
    elements: Vec<MassConstraintElement<N>>,
    kinematic_nodes: DVector<bool>,
    positions: DVector<N>,
    velocities: DVector<N>,
    accelerations: DVector<N>,
    forces: DVector<N>,
    impulses: DVector<N>,

    companion_id: usize,
    gravity_enabled: bool,
    activation: ActivationStatus<N>,
    status: BodyStatus,
    update_status: BodyUpdateStatus,
    mass: N,
    node_mass: N,
    inv_node_mass: N,
    warmstart_coeff: N,

    plasticity_threshold: N,
    plasticity_creep: N,
    plasticity_max_force: N,

    user_data: Option<Box<Any + Send + Sync>>,
}


impl<N: RealField> MassConstraintSystem<N> {
    /// Creates a new deformable surface following the mass-LengthConstraint model.
    ///
    /// The surface is initialized with a set of links corresponding to each trimesh edges.
    #[cfg(feature = "dim3")]
    pub fn from_trimesh(handle: BodyHandle, mesh: &TriMesh<N>, mass: N, stiffness: Option<N>) -> Self {
        let ndofs = mesh.points().len() * DIM;
        let mut constraints = HashMap::with_hasher(DeterministicState::new());
        let mut elements = Vec::with_capacity(mesh.faces().len());
        let mut positions = DVector::zeros(ndofs);

        for (i, pos) in positions.as_mut_slice().chunks_mut(DIM).enumerate() {
            pos.copy_from_slice(mesh.points()[i].coords.as_slice())
        }

        for (i, face) in mesh.faces().iter().enumerate() {
            let idx = face.indices * DIM;
            let elt = MassConstraintElement {
                handle: BodyPartHandle(handle, i),
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
            name: String::new(),
            handle,
            constraints: constraints.values().cloned().collect(),
            elements,
            kinematic_nodes: DVector::repeat(mesh.points().len(), false),
            positions,
            velocities: DVector::zeros(ndofs),
            accelerations: DVector::zeros(ndofs),
            forces: DVector::zeros(ndofs),
            impulses: DVector::zeros(0),
            companion_id: 0,
            activation: ActivationStatus::new_active(),
            status: BodyStatus::Dynamic,
            update_status: BodyUpdateStatus::all(),
            mass,
            node_mass,
            inv_node_mass: N::one() / node_mass,
            gravity_enabled: true,
            warmstart_coeff: na::convert(0.5),
            plasticity_threshold: N::zero(),
            plasticity_creep: N::zero(),
            plasticity_max_force: N::zero(),
            user_data: None
        }
    }

    /// Builds a mass-spring system from a polyline.
    pub fn from_polyline(handle: BodyHandle, polyline: &Polyline<N>, mass: N, stiffness: Option<N>) -> Self {
        let ndofs = polyline.points().len() * DIM;
        let mut constraints = HashMap::with_hasher(DeterministicState::new());
        let mut elements = Vec::with_capacity(polyline.edges().len());
        let mut positions = DVector::zeros(ndofs);

        for (i, pos) in positions.as_mut_slice().chunks_mut(DIM).enumerate() {
            pos.copy_from_slice(polyline.points()[i].coords.as_slice())
        }

        for (i, edge) in polyline.edges().iter().enumerate() {
            let idx = edge.indices * DIM;
            let elt = MassConstraintElement {
                handle: BodyPartHandle(handle, i),
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
            name: String::new(),
            handle,
            constraints: constraints.values().cloned().collect(),
            elements,
            kinematic_nodes: DVector::repeat(polyline.points().len(), false),
            positions,
            velocities: DVector::zeros(ndofs),
            accelerations: DVector::zeros(ndofs),
            forces: DVector::zeros(ndofs),
            impulses: DVector::zeros(constraints.len()),
            companion_id: 0,
            activation: ActivationStatus::new_active(),
            status: BodyStatus::Dynamic,
            update_status: BodyUpdateStatus::all(),
            gravity_enabled: true,
            mass,
            node_mass,
            inv_node_mass: N::one() / node_mass,
            warmstart_coeff: na::convert(0.5),
            plasticity_threshold: N::zero(),
            plasticity_creep: N::zero(),
            plasticity_max_force: N::zero(),
            user_data: None
        }
    }

    user_data_accessors!();

    /// Creates a rectangular-shaped quad.
    #[cfg(feature = "dim3")]
    pub fn quad(handle: BodyHandle, transform: &Isometry<N>, extents: &Vector2<N>, nx: usize, ny: usize, mass: N, stiffness: Option<N>) -> Self {
        let mesh = procedural::quad(extents.x, extents.y, nx, ny);
        let vertices = mesh.coords.iter().map(|pt| transform * pt).collect();
        let indices = mesh.indices.unwrap_unified().into_iter().map(|tri| na::convert(tri)).collect();
        let trimesh = TriMesh::new(vertices, indices, None);
        Self::from_trimesh(handle, &trimesh, mass, stiffness)
    }

    /// Add one constraint to this mass-constraint system.
    pub fn add_constraint(&mut self, node1: usize, node2: usize, stiffness: Option<N>) {
        assert!(node1 < self.positions.len() / DIM, "Node index out of bounds.");
        assert!(node2 < self.positions.len() / DIM, "Node index out of bounds.");
        self.update_status.set_local_inertia_changed(true);
        let key = key(node1 * DIM, node2 * DIM);
        let constraint = LengthConstraint::from_positions(key, self.positions.as_slice(), stiffness);
        self.constraints.push(constraint);
    }

    /// Generate additional constraints between nodes that are transitively neighbors.
    ///
    /// Given three nodes `a, b, c`, if a constraint exists between `a` and `b`, and between `b` and `c`,
    /// then a constraint between `a` and `c` is created if it does not already exists.
    pub fn generate_neighbor_constraints(&mut self, stiffness: Option<N>) {
        self.update_status.set_local_inertia_changed(true);

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

    /// The number of nodes of this mass-constraint system.
    pub fn num_nodes(&self) -> usize {
        self.positions.len() / DIM
    }

    /// The handle of this body.
    pub fn handle(&self) -> BodyHandle {
        self.handle
    }

    /// The total mass of this body.
    pub fn mass(&self) -> N {
        self.mass
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

    /// Restrict the specified node acceleration to always be zero so
    /// it can be controlled manually by the user at the velocity level.
    pub fn set_node_kinematic(&mut self, i: usize, is_kinematic: bool) {
        assert!(i < self.positions.len() / DIM, "Node index out of bounds.");
        self.update_status.set_status_changed(true);
        self.update_status.set_local_inertia_changed(true);
        self.kinematic_nodes[i] = is_kinematic;
    }

    /// Mark all nodes as non-kinematic.
    pub fn clear_kinematic_nodes(&mut self) {
        self.update_status.set_status_changed(true);
        self.update_status.set_local_inertia_changed(true);
        self.kinematic_nodes.fill(false)
    }

    /// Sets the plastic properties of this mass-constraint system.
    pub fn set_plasticity(&mut self, strain_threshold: N, creep: N, max_force: N) {
        self.plasticity_threshold = strain_threshold;
        self.plasticity_creep = creep;
        self.plasticity_max_force = max_force;
    }
}

impl<N: RealField> Body<N> for MassConstraintSystem<N> {
    #[inline]
    fn name(&self) -> &str {
        &self.name
    }

    #[inline]
    fn set_name(&mut self, name: String) {
        self.name = name
    }

    #[inline]
    fn gravity_enabled(&self) -> bool {
        self.gravity_enabled
    }

    #[inline]
    fn enable_gravity(&mut self, enabled: bool) {
        self.gravity_enabled = enabled
    }

    fn update_kinematics(&mut self) {
        if self.update_status.position_changed() {
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
    }

    fn update_dynamics(&mut self, _: N) {
        if self.update_status.inertia_needs_update() && self.status == BodyStatus::Dynamic && !self.is_active() {
            self.activate();
        }
    }

    fn update_acceleration(&mut self, gravity: &Vector<N>, params: &IntegrationParameters<N>) {
        self.accelerations.copy_from(&self.forces);

        if self.gravity_enabled {
            let gravity_acc = gravity;

            for i in 0..self.positions.len() / DIM {
                if !self.kinematic_nodes[i] {
                    let mut acc = self.accelerations.fixed_rows_mut::<Dim>(i * DIM);
                    acc += gravity_acc
                }
            }
        }

        // NOTE: should this be on update_dynamics?
        for constraint in &mut self.constraints {
            if let Some(stiffness) = constraint.stiffness {
                // Explicit elastic term.
                let err = constraint.length - constraint.rest_length;

                // Plastic strain.
                let total_strain = stiffness * err;
                let strain = total_strain - constraint.plastic_strain;

                if strain.abs() > self.plasticity_threshold {
                    let coeff = params.dt * (N::one() / params.dt).min(self.plasticity_creep);
                    constraint.plastic_strain += strain * coeff;
                }

                if constraint.plastic_strain.abs() > self.plasticity_max_force {
                    constraint.plastic_strain = constraint.plastic_strain.signum() * self.plasticity_max_force;
                }

                let err_with_plasticity = err - constraint.plastic_strain / stiffness;
                constraint.max_force = stiffness * err_with_plasticity.abs();

                if err_with_plasticity.abs() > params.allowed_linear_error {
                    constraint.target_vel = params.erp * err_with_plasticity / params.dt;
                } else {
                    constraint.target_vel = N::zero();
                }
            }
        }
    }

    fn clear_forces(&mut self) {
        self.forces.fill(N::zero())
    }

    fn apply_displacement(&mut self, disp: &[N]) {
        self.update_status.set_position_changed(true);
        let disp = DVectorSlice::from_slice(disp, self.positions.len());
        self.positions += disp;
    }

    fn handle(&self) -> BodyHandle {
        self.handle
    }

    fn status(&self) -> BodyStatus {
        self.status
    }

    fn set_status(&mut self, status: BodyStatus) {
        self.update_status.set_status_changed(true);
        self.status = status
    }

    fn clear_update_flags(&mut self) {
        self.update_status.clear()
    }

    fn update_status(&self) -> BodyUpdateStatus {
        self.update_status
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
        self.update_status.set_velocity_changed(true);
        let len = self.velocities.len();
        DVectorSliceMut::from_slice(self.velocities.as_mut_slice(), len)
    }

    fn integrate(&mut self, params: &IntegrationParameters<N>) {
        self.update_status.set_position_changed(true);
        self.positions.axpy(params.dt, &self.velocities, N::one())
    }

    fn activate_with_energy(&mut self, energy: N) {
        self.activation.set_energy(energy)
    }

    fn deactivate(&mut self) {
        self.update_status.clear();
        self.activation.set_energy(N::zero());
        self.velocities.fill(N::zero());
    }

    fn part(&self, id: usize) -> Option<&BodyPart<N>> {
        self.elements.get(id).map(|e| e as &BodyPart<N>)
    }

    fn deformed_positions(&self) -> Option<(DeformationsType, &[N])> {
        Some((DeformationsType::Vectors, self.positions.as_slice()))
    }

    fn deformed_positions_mut(&mut self) -> Option<(DeformationsType, &mut [N])> {
        self.update_status.set_position_changed(true);
        Some((DeformationsType::Vectors, self.positions.as_mut_slice()))
    }

    fn world_point_at_material_point(&self, part: &BodyPart<N>, point: &Point<N>) -> Point<N> {
        let elt = part.downcast_ref::<MassConstraintElement<N>>().expect("The provided body part must be a mass-constraint element");
        fem_helper::world_point_at_material_point(elt.indices, &self.positions, point)
    }

    fn position_at_material_point(&self, part: &BodyPart<N>, point: &Point<N>) -> Isometry<N> {
        let elt = part.downcast_ref::<MassConstraintElement<N>>().expect("The provided body part must be a mass-constraint element");
        let pt = fem_helper::world_point_at_material_point(elt.indices, &self.positions, point);
        Isometry::from_parts(Translation::from(pt.coords), na::one())
    }

    fn material_point_at_world_point(&self, part: &BodyPart<N>, point: &Point<N>) -> Point<N> {
        let elt = part.downcast_ref::<MassConstraintElement<N>>().expect("The provided body part must be a mass-constraint element");
        fem_helper::material_point_at_world_point(elt.indices, &self.positions, point)
    }

    fn fill_constraint_geometry(
        &self,
        part: &BodyPart<N>,
        _: usize, // FIXME: keep this parameter?
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
    fn setup_internal_velocity_constraints(&mut self, _: &DVectorSlice<N>, _: &IntegrationParameters<N>) {}

    #[inline]
    fn warmstart_internal_velocity_constraints(&mut self, dvels: &mut DVectorSliceMut<N>) {
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

                if !self.kinematic_nodes[constraint.nodes.0 / DIM] {
                    dvels.fixed_rows_mut::<Dim>(constraint.nodes.0).add_assign(&vel_correction);
                }
                if !self.kinematic_nodes[constraint.nodes.1 / DIM] {
                    dvels.fixed_rows_mut::<Dim>(constraint.nodes.1).sub_assign(&vel_correction);
                }
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
            let kinematic1 = self.kinematic_nodes[constraint.nodes.0 / DIM];
            let kinematic2 = self.kinematic_nodes[constraint.nodes.1 / DIM];

            if kinematic1 && kinematic2 {
                continue;
            }

            let v0 = self.velocities.fixed_rows::<Dim>(constraint.nodes.0) + dvels.fixed_rows::<Dim>(constraint.nodes.0);
            let v1 = self.velocities.fixed_rows::<Dim>(constraint.nodes.1) + dvels.fixed_rows::<Dim>(constraint.nodes.1);

            let dvel = (v1 - v0).dot(&constraint.dir);
            let dlambda;
            let denom = if kinematic1 || kinematic2 {
                self.inv_node_mass
            } else {
                self.inv_node_mass + self.inv_node_mass
            };

            if constraint.stiffness.is_some() {
                let curr_impulse = self.impulses[i];
                let dimpulse = (dvel + constraint.target_vel) / denom;
                let new_impulse = na::clamp(curr_impulse + dimpulse, -constraint.max_force, constraint.max_force);
                dlambda = new_impulse - curr_impulse;
                self.impulses[i] = new_impulse;
            } else {
                dlambda = dvel / denom;
                self.impulses[i] += dlambda;
            }

            let vel_correction = *constraint.dir * (dlambda * self.inv_node_mass);

            if !kinematic1 {
                dvels.fixed_rows_mut::<Dim>(constraint.nodes.0).add_assign(&vel_correction);
            }
            if !kinematic2 {
                dvels.fixed_rows_mut::<Dim>(constraint.nodes.1).sub_assign(&vel_correction);
            }
        }
    }

    #[inline]
    fn step_solve_internal_position_constraints(&mut self, params: &IntegrationParameters<N>) {
        for constraint in &mut self.constraints {
            if constraint.stiffness.is_none() {
                let kinematic1 = self.kinematic_nodes[constraint.nodes.0 / DIM];
                let kinematic2 = self.kinematic_nodes[constraint.nodes.1 / DIM];

                if kinematic1 && kinematic2 {
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
                    let coeff = if kinematic1 || kinematic2 {
                        N::one()
                    } else {
                        na::convert(0.5)
                    };
                    let shift = *constraint.dir * (clamped_error * coeff);

                    if !kinematic1 {
                        self.positions.fixed_rows_mut::<Dim>(constraint.nodes.0).add_assign(&shift);
                    }
                    if !kinematic2 {
                        self.positions.fixed_rows_mut::<Dim>(constraint.nodes.1).sub_assign(&shift);
                    }
                }
            }
        }
    }


    fn apply_force_at_local_point(&mut self, part_id: usize, force: &Vector<N>, point: &Point<N>, force_type: ForceType, auto_wake_up: bool) {
        if self.status != BodyStatus::Dynamic {
            return;
        }

        if auto_wake_up {
            self.activate()
        }

        let element = &self.elements[part_id];
        let indices = element.indices.as_slice();

        #[cfg(feature = "dim2")]
        let forces = [
            force * (N::one() - point.x - point.y),
            force * point.x,
            force * point.y,
        ];
        #[cfg(feature = "dim3")]
        let forces = [
            force * (N::one() - point.x - point.y - point.z),
            force * point.x,
            force * point.y,
            force * point.z,
        ];

        match force_type {
            ForceType::Force => {
                for i in 0..indices.len() {
                    if !self.kinematic_nodes[indices[i] / DIM] {
                        self.forces.fixed_rows_mut::<Dim>(indices[i]).add_assign(&forces[i]);
                    }
                }
            }
            ForceType::Impulse => {
                for i in 0..indices.len() {
                    if !self.kinematic_nodes[indices[i] / DIM] {
                        self.velocities.fixed_rows_mut::<Dim>(indices[i]).add_assign(forces[i] * self.inv_node_mass);
                    }
                }
            }
            ForceType::AccelerationChange => {
                for i in 0..indices.len() {
                    if !self.kinematic_nodes[indices[i] / DIM] {
                        self.forces.fixed_rows_mut::<Dim>(indices[i]).add_assign(forces[i] * self.node_mass);
                    }
                }
            }
            ForceType::VelocityChange => {
                for i in 0..indices.len() {
                    if !self.kinematic_nodes[indices[i] / DIM] {
                        self.velocities.fixed_rows_mut::<Dim>(indices[i]).add_assign(forces[i]);
                    }
                }
            }
        }
    }

    fn apply_force(&mut self, part_id: usize, force: &Force<N>, force_type: ForceType, auto_wake_up: bool) {
        let dim = self.elements[part_id].indices.as_slice().len();
        let inv_dim: N = na::convert(1.0 / dim as f64);
        let barycenter = Point::from(Vector::repeat(inv_dim));
        self.apply_force_at_local_point(part_id, &force.linear, &barycenter, force_type, auto_wake_up)
    }

    fn apply_local_force(&mut self, part_id: usize, force: &Force<N>, force_type: ForceType, auto_wake_up: bool) {
        // FIXME: compute an approximate rotation for the conserned element (just like the FEM bodies)?
        self.apply_force(part_id, &force, force_type, auto_wake_up);
    }

    fn apply_force_at_point(&mut self, part_id: usize, force: &Vector<N>, point: &Point<N>, force_type: ForceType, auto_wake_up: bool) {
        let local_point = self.material_point_at_world_point(&self.elements[part_id], point);
        self.apply_force_at_local_point(part_id, &force, &local_point, force_type, auto_wake_up)
    }

    fn apply_local_force_at_point(&mut self, part_id: usize, force: &Vector<N>, point: &Point<N>, force_type: ForceType, auto_wake_up: bool) {
        // FIXME: compute an approximate rotation for the conserned element (just like the FEM bodies)?
        let local_point = self.material_point_at_world_point(&self.elements[part_id], point);
        self.apply_force_at_local_point(part_id, &force, &local_point, force_type, auto_wake_up);
    }


    fn apply_local_force_at_local_point(&mut self, part_id: usize, force: &Vector<N>, point: &Point<N>, force_type: ForceType, auto_wake_up: bool) {
        // FIXME: compute an approximate rotation for the conserned element (just like the FEM bodies)?
        self.apply_force_at_local_point(part_id, &force, &point, force_type, auto_wake_up);
    }
}


impl<N: RealField> BodyPart<N> for MassConstraintElement<N> {
    fn part_handle(&self) -> BodyPartHandle {
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
}




enum MassConstraintSystemDescGeometry<'a, N: RealField> {
    Quad(usize, usize),
    Polyline(&'a Polyline<N>),
    #[cfg(feature = "dim3")]
    TriMesh(&'a TriMesh<N>),
}

/// A builder of a mass-constraint system.
pub struct MassConstraintSystemDesc<'a, N: RealField> {
    name: String,
    user_data: Option<UserDataBox>,
    geom: MassConstraintSystemDescGeometry<'a, N>,
    scale: Vector<N>,
    position: Isometry<N>,
    stiffness: Option<N>,
    sleep_threshold: Option<N>,
//    damping_ratio: N,
    mass: N,
    plasticity: (N, N, N),
    kinematic_nodes: Vec<usize>,
    status: BodyStatus,
    collider_enabled: bool,
    gravity_enabled: bool,
}

impl<'a, N: RealField> MassConstraintSystemDesc<'a, N> {
    fn with_geometry(geom: MassConstraintSystemDescGeometry<'a, N>) -> Self {
        MassConstraintSystemDesc {
            name: String::new(),
            user_data: None,
            gravity_enabled: true,
            geom,
            scale: Vector::repeat(N::one()),
            position: Isometry::identity(),
            stiffness: Some(na::convert(1.0e3)),
            sleep_threshold: Some(ActivationStatus::default_threshold()),
//            damping_ratio: na::convert(0.2),
            mass: N::one(),
            plasticity: (N::zero(), N::zero(), N::zero()),
            kinematic_nodes: Vec::new(),
            status: BodyStatus::Dynamic,
            collider_enabled: false
        }
    }

    user_data_desc_accessors!();

    /// Create a mass-constraints system form a triangle mesh.
    #[cfg(feature = "dim3")]
    pub fn from_trimesh(mesh: &'a TriMesh<N>) -> Self {
        Self::with_geometry(MassConstraintSystemDescGeometry::TriMesh(mesh))
    }

    /// Create a mass-constraints system form a polygonal line.
    pub fn from_polyline(polyline: &'a Polyline<N>) -> Self {
        Self::with_geometry(MassConstraintSystemDescGeometry::Polyline(polyline))
    }

    /// Create a quad-shaped body.
    pub fn quad(subdiv_x: usize, subdiv_y: usize) -> Self {
        Self::with_geometry(MassConstraintSystemDescGeometry::Quad(subdiv_x, subdiv_y))
    }

    /// Mark all nodes as non-kinematic.
    pub fn clear_kinematic_nodes(&mut self) -> &mut Self {
        self.kinematic_nodes.clear();
        self
    }

    desc_custom_setters!(
        self.plasticity, set_plasticity, strain_threshold: N, creep: N, max_force: N | { self.plasticity = (strain_threshold, creep, max_force) }
        self.kinematic_nodes, set_nodes_kinematic, nodes: &[usize] | { self.kinematic_nodes.extend_from_slice(nodes) }
        self.translation, set_translation, vector: Vector<N> | { self.position.translation.vector = vector }
        self.name, set_name, name: String | { self.name = name }
    );

    desc_setters!(
        gravity_enabled, enable_gravity, gravity_enabled: bool
        collider_enabled, set_collider_enabled, collider_enabled: bool
        scale, set_scale, scale: Vector<N>
        stiffness, set_stiffness, stiffness: Option<N>
        sleep_threshold, set_sleep_threshold, sleep_threshold: Option<N>
//        damping_ratio, set_damping_ratio, damping_ratio: N
        mass, set_mass, mass: N
        status, set_status, status: BodyStatus
        position, set_position, position: Isometry<N>
    );

    desc_custom_getters!(
        self.get_plasticity_strain_threshold: N | { self.plasticity.0 }
        self.get_plasticity_creep: N | { self.plasticity.1 }
        self.get_plasticity_max_force: N | { self.plasticity.2 }
        self.get_kinematic_nodes: &[usize] | { &self.kinematic_nodes[..] }
        self.get_translation: &Vector<N> | { &self.position.translation.vector }
        self.get_name: &str | { &self.name }
    );

    desc_getters!(
        [val] is_gravity_enabled -> gravity_enabled: bool
        [val] get_stiffness -> stiffness: Option<N>
        [val] get_sleep_threshold -> sleep_threshold: Option<N>
//        [val] get_damping_ratio -> damping_ratio: N
        [val] get_mass -> mass: N
        [val] get_status -> status: BodyStatus
        [val] is_collider_enabled -> collider_enabled: bool
        [ref] get_position -> position: Isometry<N>
        [ref] get_scale -> scale: Vector<N>
    );

    /// Build a mass-constraint system.
    pub fn build<'w>(&self, world: &'w mut World<N>) -> &'w mut MassConstraintSystem<N> {
        world.add_body(self)
    }
}

impl<'a, N: RealField> BodyDesc<N> for MassConstraintSystemDesc<'a, N> {
    type Body = MassConstraintSystem<N>;

    fn build_with_handle(&self, cworld: &mut ColliderWorld<N>, handle: BodyHandle) -> MassConstraintSystem<N> {
        let mut vol = match self.geom {
            #[cfg(feature = "dim3")]
            MassConstraintSystemDescGeometry::Quad(nx, ny) => {
                MassConstraintSystem::quad(
                    handle, &self.position, &self.scale.xy(), nx, ny,
                    self.mass, self.stiffness)
            }
            #[cfg(feature = "dim2")]
            MassConstraintSystemDescGeometry::Quad(nx, ny) => {
                let mut polyline = Polyline::quad(nx, ny);
                polyline.scale_by(&self.scale);
                polyline.transform_by(&self.position);

                let vol = MassConstraintSystem::from_polyline(
                    handle, &polyline, self.mass, self.stiffness);


                if self.collider_enabled {
                    let _ = DeformableColliderDesc::new(ShapeHandle::new(polyline.clone()))
                        .build_with_infos(&vol, cworld);
                }

                vol
            }
            MassConstraintSystemDescGeometry::Polyline(polyline) => {
                let mut polyline = polyline.clone();
                polyline.scale_by(&self.scale);
                polyline.transform_by(&self.position);

                let vol = MassConstraintSystem::from_polyline(
                    handle, &polyline, self.mass, self.stiffness);
                if self.collider_enabled {
                    let _ = DeformableColliderDesc::new(ShapeHandle::new(polyline))
                        .build_with_infos(&vol, cworld);
                }

                vol
            }
            #[cfg(feature = "dim3")]
            MassConstraintSystemDescGeometry::TriMesh(trimesh) => {
                let mut trimesh = trimesh.clone();
                trimesh.scale_by(&self.scale);
                trimesh.transform_by(&self.position);

                let vol = MassConstraintSystem::from_trimesh(handle, &trimesh, self.mass, self.stiffness);
                if self.collider_enabled {
                    let _ = DeformableColliderDesc::new(ShapeHandle::new(trimesh.clone()))
                        .build_with_infos(&vol, cworld);
                }

                vol
            }
        };

        vol.set_deactivation_threshold(self.sleep_threshold);
        vol.set_plasticity(self.plasticity.0, self.plasticity.1, self.plasticity.2);
        vol.enable_gravity(self.gravity_enabled);
        vol.set_name(self.name.clone());
        vol.set_status(self.status);
        let _ = vol.set_user_data(self.user_data.as_ref().map(|data| data.0.to_any()));

        for i in &self.kinematic_nodes {
            vol.set_node_kinematic(*i, true)
        }

        vol
    }
}