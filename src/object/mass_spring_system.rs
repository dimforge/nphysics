use std::ops::{AddAssign, SubAssign};
use std::iter;
use std::collections::{HashMap, HashSet};
use std::marker::PhantomData;
use std::any::Any;
use either::Either;

use na::{self, Real, DMatrix, DVector, DVectorSlice, DVectorSliceMut, Cholesky, Dynamic, MatrixN, Unit};
#[cfg(feature = "dim3")]
use na::Vector2;
use ncollide::utils::DeterministicState;
#[cfg(feature = "dim3")]
use ncollide::procedural;
use ncollide::shape::{DeformationsType, Polyline, ShapeHandle};
#[cfg(feature = "dim3")]
use ncollide::shape::TriMesh;

use crate::object::{Body, BodyPart, BodyHandle, BodyPartHandle, BodyStatus, ActivationStatus, FiniteElementIndices,
                    BodyDesc, DeformableColliderDesc};
use crate::solver::{IntegrationParameters, ForceDirection};
use crate::math::{Force, Inertia, Velocity, Vector, Point, Isometry, DIM, Dim, Translation};
use crate::object::fem_helper;
use crate::world::{World, ColliderWorld};

/// An element of the mass-spring system.
#[derive(Clone)]
pub struct MassSpringElement<N: Real> {
    handle: BodyPartHandle,
    indices: FiniteElementIndices,
    phantom: PhantomData<N>
}

#[derive(Clone)]
struct Spring<N: Real> {
    nodes: (usize, usize),
    // Should be Unit<Vector<N>>, but can be zero.
    dir: Unit<Vector<N>>,
    length: N,
    rest_length: N,
    stiffness: N,
    damping_ratio: N,
    plastic_strain: N,
}

impl<N: Real> Spring<N> {
    fn from_positions(nodes: (usize, usize), positions: &[N], stiffness: N, damping_ratio: N) -> Self {
        let p0 = Point::from_slice(&positions[nodes.0..nodes.0 + DIM]);
        let p1 = Point::from_slice(&positions[nodes.1..nodes.1 + DIM]);
        let rest_length = na::distance(&p0, &p1);

        Spring {
            nodes,
            dir: Unit::new_normalize(p1 - p0),
            length: rest_length,
            rest_length,
            stiffness,
            damping_ratio,
            plastic_strain: N::zero()
        }
    }
}

/// A deformable surface using a mass-spring model with triangular elements.
pub struct MassSpringSystem<N: Real> {
    handle: BodyHandle,
    springs: Vec<Spring<N>>,
    elements: Vec<MassSpringElement<N>>,
    kinematic_nodes: DVector<bool>,
    positions: DVector<N>,
    velocities: DVector<N>,
    accelerations: DVector<N>,
    augmented_mass: DMatrix<N>,
    inv_augmented_mass: Cholesky<N, Dynamic>,

    companion_id: usize,
    activation: ActivationStatus<N>,
    status: BodyStatus,
    mass: N,
    node_mass: N,

    plasticity_threshold: N,
    plasticity_creep: N,
    plasticity_max_force: N,

    user_data: Option<Box<Any + Send + Sync>>,
}

fn key(i: usize, j: usize) -> (usize, usize) {
    if i <= j {
        (i, j)
    } else {
        (j, i)
    }
}

impl<N: Real> MassSpringSystem<N> {
    /// Creates a new deformable surface following the mass-spring model.
    ///
    /// The surface is initialized with a set of links corresponding to each trimesh edges.
    #[cfg(feature = "dim3")]
    fn from_trimesh(handle: BodyHandle, mesh: &TriMesh<N>, mass: N, stiffness: N, damping_ratio: N) -> Self {
        let ndofs = mesh.points().len() * DIM;
        let mut springs = HashMap::with_hasher(DeterministicState::new());
        let mut elements = Vec::with_capacity(mesh.faces().len());
        let mut positions = DVector::zeros(ndofs);

        for (i, pos) in positions.as_mut_slice().chunks_mut(DIM).enumerate() {
            pos.copy_from_slice(mesh.points()[i].coords.as_slice())
        }

        for (i, face) in mesh.faces().iter().enumerate() {
            let idx = face.indices * DIM;
            let elt = MassSpringElement {
                handle: BodyPartHandle(handle, i),
                indices: FiniteElementIndices::Triangle(idx),
                phantom: PhantomData
            };

            elements.push(elt);

            let _ = springs.entry(key(idx.x, idx.y)).or_insert_with(|| {
                Spring::from_positions((idx.x, idx.y), positions.as_slice(), stiffness, damping_ratio)
            });
            let _ = springs.entry(key(idx.y, idx.z)).or_insert_with(|| {
                Spring::from_positions((idx.y, idx.z), positions.as_slice(), stiffness, damping_ratio)
            });
            let _ = springs.entry(key(idx.z, idx.x)).or_insert_with(|| {
                Spring::from_positions((idx.z, idx.x), positions.as_slice(), stiffness, damping_ratio)
            });
        }

        let node_mass = mass / na::convert((ndofs / DIM) as f64);

        MassSpringSystem {
            handle,
            springs: springs.values().cloned().collect(),
            elements,
            kinematic_nodes: DVector::repeat(ndofs / DIM, false),
            positions,
            velocities: DVector::zeros(ndofs),
            accelerations: DVector::zeros(ndofs),
            augmented_mass: DMatrix::zeros(ndofs, ndofs),
            inv_augmented_mass: Cholesky::new(DMatrix::zeros(0, 0)).unwrap(),
            companion_id: 0,
            activation: ActivationStatus::new_active(),
            status: BodyStatus::Dynamic,
            mass,
            node_mass,
            plasticity_max_force: N::zero(),
            plasticity_creep: N::zero(),
            plasticity_threshold: N::zero()
        }
    }

    user_data_accessors!();

    /// Builds a mass-spring system from a polyline.
    fn from_polyline(handle: BodyHandle, polyline: &Polyline<N>, mass: N, stiffness: N, damping_ratio: N) -> Self {
        let ndofs = polyline.points().len() * DIM;
        let mut springs = HashMap::with_hasher(DeterministicState::new());
        let mut elements = Vec::with_capacity(polyline.edges().len());
        let mut positions = DVector::zeros(ndofs);

        for (i, pos) in positions.as_mut_slice().chunks_mut(DIM).enumerate() {
            pos.copy_from_slice(polyline.points()[i].coords.as_slice())
        }

        for (i, edge) in polyline.edges().iter().enumerate() {
            let idx = edge.indices * DIM;
            let elt = MassSpringElement {
                handle: BodyPartHandle(handle, i),
                indices: FiniteElementIndices::Segment(idx),
                phantom: PhantomData
            };

            elements.push(elt);

            let _ = springs.entry(key(idx.x, idx.y)).or_insert_with(|| {
                Spring::from_positions((idx.x, idx.y), positions.as_slice(), stiffness, damping_ratio)
            });
        }

        let node_mass = mass / na::convert((ndofs / DIM) as f64);
        println!("Number of nodes: {}, of springs: {}", positions.len() / DIM, springs.len());

        MassSpringSystem {
            handle,
            springs: springs.values().cloned().collect(),
            kinematic_nodes: DVector::repeat(ndofs / DIM, false),
            elements,
            positions,
            velocities: DVector::zeros(ndofs),
            accelerations: DVector::zeros(ndofs),
            augmented_mass: DMatrix::zeros(ndofs, ndofs),
            inv_augmented_mass: Cholesky::new(DMatrix::zeros(0, 0)).unwrap(),
            companion_id: 0,
            activation: ActivationStatus::new_active(),
            status: BodyStatus::Dynamic,
            mass,
            node_mass,
            plasticity_max_force: N::zero(),
            plasticity_creep: N::zero(),
            plasticity_threshold: N::zero(),
            user_data: None
        }
    }

    /// Creates a rectangular quad.
    #[cfg(feature = "dim3")]
    fn quad(handle: BodyHandle, transform: &Isometry<N>, extents: &Vector2<N>, nx: usize, ny: usize, mass: N, stiffness: N, damping_ratio: N) -> Self {
        let mesh = procedural::quad(extents.x, extents.y, nx, ny);
        let vertices = mesh.coords.iter().map(|pt| transform * pt).collect();
        let indices = mesh.indices.unwrap_unified().into_iter().map(|tri| na::convert(tri)).collect();
        let trimesh = TriMesh::new(vertices, indices, None);
        Self::from_trimesh(handle,&trimesh, mass, stiffness, damping_ratio)
    }

    pub fn num_nodes(&self) -> usize {
        self.positions.len() / DIM
    }

    pub fn handle(&self) -> BodyHandle {
        self.handle
    }

    /// Generate additional springs between nodes that are transitively neighbors.
    ///
    /// Given three nodes `a, b, c`, if a spring exists between `a` and `b`, and between `b` and `c`,
    /// then a spring between `a` and `c` is created if it does not already exists.
    pub fn generate_neighbor_springs(&mut self, stiffness: N, damping_ratio: N) {
        let mut neighbor_list: Vec<_> = iter::repeat(Vec::new()).take(self.positions.len() / DIM).collect();
        let mut existing_springs = HashSet::with_hasher(DeterministicState::new());

        // Build neighborhood list.
        for spring in &self.springs {
            let key = key(spring.nodes.0, spring.nodes.1);
            neighbor_list[key.0 / DIM].push(key.1 / DIM);
            let _ = existing_springs.insert(key);
        }

        // Build springs.
        for (i, nbhs) in neighbor_list.iter().enumerate() {
            for nbh in nbhs {
                for transitive_nbh in &neighbor_list[*nbh] {
                    let key = key(i * DIM, *transitive_nbh * DIM);

                    if existing_springs.insert(key) {
                        let spring =
                            Spring::from_positions(key, self.positions.as_slice(), stiffness, damping_ratio);
                        self.springs.push(spring);
                    }
                }
            }
        }
    }

    /// Add one spring to this mass-spring system.
    pub fn add_spring(&mut self, node1: usize, node2: usize, stiffness: N, damping_ratio: N) {
        assert!(node1 < self.positions.len() / DIM, "Node index out of bounds.");
        assert!(node2 < self.positions.len() / DIM, "Node index out of bounds.");
        let key = key(node1 * DIM, node2 * DIM);
        let spring = Spring::from_positions(key, self.positions.as_slice(), stiffness, damping_ratio);
        self.springs.push(spring);
    }

    /// Restrict the specified node acceleration to always be zero so
    /// it can be controlled manually by the user at the velocity level.
    pub fn set_node_kinematic(&mut self, i: usize, is_kinematic: bool) {
        assert!(i < self.positions.len() / DIM, "Node index out of bounds.");
        self.kinematic_nodes[i] = is_kinematic;
    }

    /// Sets the plastic properties of this mass-spring system.
    pub fn set_plasticity(&mut self, strain_threshold: N, creep: N, max_force: N) {
        self.plasticity_threshold = strain_threshold;
        self.plasticity_creep = creep;
        self.plasticity_max_force = max_force;
    }

    fn update_augmented_mass_and_forces(&mut self, gravity: &Vector<N>, params: &IntegrationParameters<N>) {
        self.augmented_mass.fill_diagonal(self.node_mass);

        for spring in &mut self.springs {
            let kinematic1 = self.kinematic_nodes[spring.nodes.0 / DIM];
            let kinematic2 = self.kinematic_nodes[spring.nodes.1 / DIM];

            if kinematic1 && kinematic2 {
                continue;
            }

            /*
             *
             * Plastic strain.
             *
             */
            // Compute plastic strain.
            let total_strain = spring.stiffness * (spring.length - spring.rest_length);
            let strain = total_strain - spring.plastic_strain;

            if strain.abs() > self.plasticity_threshold {
                let coeff = params.dt * (N::one() / params.dt).min(self.plasticity_creep);
                spring.plastic_strain += strain * coeff;
            }

            if spring.plastic_strain.abs() > self.plasticity_max_force {
                spring.plastic_strain = spring.plastic_strain.signum() * self.plasticity_max_force;
            }

            /*
             * Elastic strain.
             */
            let damping = spring.damping_ratio * (spring.stiffness * self.node_mass).sqrt() * na::convert(2.0);
            let v0 = self.velocities.fixed_rows::<Dim>(spring.nodes.0);
            let v1 = self.velocities.fixed_rows::<Dim>(spring.nodes.1);

            let ldot = v1 - v0;
            let l = *spring.dir;

            // Explicit elastic term - plastic term.
            let coeff = spring.stiffness * (spring.length - spring.rest_length) + damping * ldot.dot(&l);
            let f0 = l * (coeff - spring.plastic_strain);

            if !kinematic1 {
                self.accelerations.fixed_rows_mut::<Dim>(spring.nodes.0).add_assign(&f0);
            }
            if !kinematic2 {
                self.accelerations.fixed_rows_mut::<Dim>(spring.nodes.1).sub_assign(&f0);
            }


            if spring.length != N::zero() {
                /*
                 *
                 * Stiffness matrix contribution.
                 *
                 */
                let ll = l * l.transpose();
                let one_minus_ll = MatrixN::<N, Dim>::identity() - ll;
                let stiffness = one_minus_ll / spring.length + ll * one_minus_ll * (damping / spring.length) + ll * spring.stiffness;
                let forward_f0 = stiffness * (ldot * params.dt);

                // Add the contributions to the forces.
                if !kinematic1 {
                    self.accelerations.fixed_rows_mut::<Dim>(spring.nodes.0).add_assign(&forward_f0);
                }
                if !kinematic2 {
                    self.accelerations.fixed_rows_mut::<Dim>(spring.nodes.1).sub_assign(&forward_f0);
                }

                // Damping matrix contribution.
                let damping_dt = (l * l.transpose()) * (damping * params.dt);

                // Add to the mass matrix.
                let damping_stiffness = damping_dt + stiffness * (params.dt * params.dt);

                if !kinematic1 {
                    self.augmented_mass.fixed_slice_mut::<Dim, Dim>(spring.nodes.0, spring.nodes.0).add_assign(&damping_stiffness);
                }
                if !kinematic2 {
                    self.augmented_mass.fixed_slice_mut::<Dim, Dim>(spring.nodes.1, spring.nodes.1).add_assign(&damping_stiffness);
                }
                if !kinematic1 && !kinematic2 {
                    // FIXME: we don't need to fill both because Cholesky won't read tho upper triangle.
                    self.augmented_mass.fixed_slice_mut::<Dim, Dim>(spring.nodes.0, spring.nodes.1).sub_assign(&damping_stiffness);
                    self.augmented_mass.fixed_slice_mut::<Dim, Dim>(spring.nodes.1, spring.nodes.0).sub_assign(&damping_stiffness);
                }
            }
        }

        /*
         * Add forces due to gravity and set the mass matrix diagonal
         * to the identity for kinematic nodes.
         */
        let gravity_force = gravity * self.node_mass;

        for i in 0..self.positions.len() / DIM {
            let idof = i * DIM;

            if !self.kinematic_nodes[i] {
                let mut acc = self.accelerations.fixed_rows_mut::<Dim>(idof);
                acc += gravity_force
            } else {
                self.augmented_mass.fixed_slice_mut::<Dim, Dim>(idof, idof).fill_diagonal(N::one());
            }
        }

        /*
         * Invert the augmented mass.
         */
        self.inv_augmented_mass = Cholesky::new(self.augmented_mass.clone()).unwrap();
        self.inv_augmented_mass.solve_mut(&mut self.accelerations);
    }
}

impl<N: Real> Body<N> for MassSpringSystem<N> {
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
    }

    fn clear_dynamics(&mut self) {
        self.accelerations.fill(N::zero());
        self.augmented_mass.fill(N::zero());
    }

    fn apply_displacement(&mut self, disp: &[N]) {
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
        self.positions.axpy(params.dt, &self.velocities, N::one());
    }

    fn activate_with_energy(&mut self, energy: N) {
        self.activation.set_energy(energy)
    }

    fn deactivate(&mut self) {
        self.activation.set_energy(N::zero());
        self.velocities.fill(N::zero());
    }

    fn part(&self, id: usize) -> Option<&BodyPart<N>> {
        self.elements.get(id).map(|e| e as &BodyPart<N>)
    }

    fn part_mut(&mut self, id: usize) -> Option<&mut BodyPart<N>> {
        self.elements.get_mut(id).map(|e| e as &mut BodyPart<N>)
    }

    fn deformed_positions(&self) -> Option<(DeformationsType, &[N])> {
        Some((DeformationsType::Vectors, self.positions.as_slice()))
    }

    fn deformed_positions_mut(&mut self) -> Option<(DeformationsType, &mut [N])> {
        Some((DeformationsType::Vectors, self.positions.as_mut_slice()))
    }

    fn world_point_at_material_point(&self, part: &BodyPart<N>, point: &Point<N>) -> Point<N> {
        let elt = part.downcast_ref::<MassSpringElement<N>>().expect("The provided body part must be a mass-spring element");
        fem_helper::world_point_at_material_point(elt.indices, &self.positions, point)
    }

    fn position_at_material_point(&self, part: &BodyPart<N>, point: &Point<N>) -> Isometry<N> {
        let elt = part.downcast_ref::<MassSpringElement<N>>().expect("The provided body part must be a mass-spring element");
        let pt = fem_helper::world_point_at_material_point(elt.indices, &self.positions, point);
        Isometry::from_parts(Translation::from_vector(pt.coords), na::one())
    }

    fn material_point_at_world_point(&self, part: &BodyPart<N>, point: &Point<N>) -> Point<N> {
        let elt = part.downcast_ref::<MassSpringElement<N>>().expect("The provided body part must be a mass-spring element");
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
        let elt = part.downcast_ref::<MassSpringElement<N>>().expect("The provided body part must be a mass-spring element");
        fem_helper::fill_contact_geometry_fem(
            self.ndofs(),
            self.status,
            elt.indices,
            &self.positions,
            &self.velocities,
            &self.kinematic_nodes,
            Either::Right(&self.inv_augmented_mass),
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
        false
    }

    #[inline]
    fn setup_internal_velocity_constraints(&mut self, _: &mut DVectorSliceMut<N>) {}

    #[inline]
    fn step_solve_internal_velocity_constraints(&mut self, _: &mut DVectorSliceMut<N>) {}

    #[inline]
    fn step_solve_internal_position_constraints(&mut self, _: &IntegrationParameters<N>) {}
}


impl<N: Real> BodyPart<N> for MassSpringElement<N> {
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

    fn apply_force(&mut self, _force: &Force<N>) {
        unimplemented!()
    }
}


enum MassSpringSystemDescGeometry<'a, N: Real> {
    Quad(usize, usize),
    Polyline(&'a Polyline<N>),
    #[cfg(feature = "dim3")]
    TriMesh(&'a TriMesh<N>),
}

pub struct MassSpringSystemDesc<'a, N: Real> {
    geom: MassSpringSystemDescGeometry<'a, N>,
    scale: Vector<N>,
    position: Isometry<N>,
    stiffness: N,
    sleep_threshold: Option<N>,
    damping_ratio: N,
    mass: N,
    plasticity: (N, N, N),
    kinematic_nodes: Vec<usize>,
    status: BodyStatus,
    collider_enabled: bool
}

impl<'a, N: Real> MassSpringSystemDesc<'a, N> {
    fn with_geometry(geom: MassSpringSystemDescGeometry<'a, N>) -> Self {
        MassSpringSystemDesc {
            geom,
            scale: Vector::repeat(N::one()),
            position: Isometry::identity(),
            stiffness: na::convert(1.0e3),
            sleep_threshold: Some(ActivationStatus::default_threshold()),
            damping_ratio: na::convert(0.2),
            mass: N::one(),
            plasticity: (N::zero(), N::zero(), N::zero()),
            kinematic_nodes: Vec::new(),
            status: BodyStatus::Dynamic,
            collider_enabled: false
        }
    }

    #[cfg(feature = "dim3")]
    pub fn from_trimesh(mesh: &'a TriMesh<N>) -> Self {
        Self::with_geometry(MassSpringSystemDescGeometry::TriMesh(mesh))
    }

    pub fn from_polyline(polyline: &'a Polyline<N>) -> Self {
        Self::with_geometry(MassSpringSystemDescGeometry::Polyline(polyline))
    }

    pub fn quad(subdiv_x: usize, subdiv_y: usize) -> Self {
        Self::with_geometry(MassSpringSystemDescGeometry::Quad(subdiv_x, subdiv_y))
    }

    pub fn clear_kinematic_nodes(&mut self) -> &mut Self {
        self.kinematic_nodes.clear();
        self
    }

    desc_custom_setters!(
        self.with_plasticity, set_plasticity, strain_threshold: N, creep: N, max_force: N | { self.plasticity = (strain_threshold, creep, max_force) }
        self.with_kinematic_nodes, set_kinematic_nodes, nodes: &[usize] | { self.kinematic_nodes.extend_from_slice(nodes) }
        self.with_translation, set_translation, vector: Vector<N> | { self.position.translation.vector = vector }
    );

    desc_setters!(
        with_collider_enabled, set_collider_enabled, collider_enabled: bool
        with_scale, set_scale, scale: Vector<N>
        with_stiffness, set_stiffness, stiffness: N
        with_sleep_threshold, set_sleep_threshold, sleep_threshold: Option<N>
        with_damping_ratio, set_damping_ratio, damping_ratio: N
        with_mass, set_mass, mass: N
        with_status, set_status, status: BodyStatus
        with_position, set_position, position: Isometry<N>
    );

    desc_custom_getters!(
        self.plasticity_strain_threshold: N | { self.plasticity.0 }
        self.plasticity_creep: N | { self.plasticity.1 }
        self.plasticity_max_force: N | { self.plasticity.2 }
        self.kinematic_nodes: &[usize] | { &self.kinematic_nodes[..] }
        self.translation: &Vector<N> | { &self.position.translation.vector }
    );

    desc_getters!(
        [val] stiffness: N
        [val] sleep_threshold: Option<N>
        [val] damping_ratio: N
        [val] mass: N
        [val] status: BodyStatus
        [val] collider_enabled: bool
        [ref] position: Isometry<N>
        [ref] scale: Vector<N>
    );

    pub fn build<'w>(&self, world: &'w mut World<N>) -> &'w mut MassSpringSystem<N> {
        world.add_body(self)
    }
}

impl<'a, N: Real> BodyDesc<N> for MassSpringSystemDesc<'a, N> {
    type Body = MassSpringSystem<N>;

    fn build_with_handle(&self, cworld: &mut ColliderWorld<N>, handle: BodyHandle) -> MassSpringSystem<N> {
        let mut vol = match self.geom {
            #[cfg(feature = "dim3")]
            MassSpringSystemDescGeometry::Quad(nx, ny) => {
                MassSpringSystem::quad(
                    handle, &self.position, &self.scale.xy(), nx, ny,
                    self.mass, self.stiffness, self.damping_ratio)
            }
            #[cfg(feature = "dim2")]
            MassSpringSystemDescGeometry::Quad(nx, ny) => {
                let mut polyline = Polyline::quad(nx, ny);
                polyline.scale_by(&self.scale);
                polyline.transform_by(&self.position);

                let vol = MassSpringSystem::from_polyline(
                    handle, &polyline, self.mass, self.stiffness, self.damping_ratio);


                if self.collider_enabled {
                    let _ = DeformableColliderDesc::new(ShapeHandle::new(polyline.clone()))
                        .build_with_infos(&vol, cworld);
                }

                vol
            }
            MassSpringSystemDescGeometry::Polyline(polyline) => {
                let mut polyline = polyline.clone();
                polyline.scale_by(&self.scale);
                polyline.transform_by(&self.position);

                let vol = MassSpringSystem::from_polyline(
                    handle, &polyline, self.mass, self.stiffness, self.damping_ratio);
                if self.collider_enabled {
                    let _ = DeformableColliderDesc::new(ShapeHandle::new(polyline))
                        .build_with_infos(&vol, cworld);
                }

                vol
            }
            #[cfg(feature = "dim3")]
            MassSpringSystemDescGeometry::TriMesh(trimesh) => {
                let mut trimesh = trimesh.clone();
                trimesh.scale_by(&self.scale);
                trimesh.transform_by(&self.position);

                let vol = MassSpringSystem::from_trimesh(handle, &trimesh, self.mass, self.stiffness, self.damping_ratio);
                if self.collider_enabled {
                    let _ = DeformableColliderDesc::new(ShapeHandle::new(trimesh.clone()))
                        .build_with_infos(&vol, cworld);
                }

                vol
            }
        };

        vol.set_deactivation_threshold(self.sleep_threshold);
        vol.set_plasticity(self.plasticity.0, self.plasticity.1, self.plasticity.2);

        for i in &self.kinematic_nodes {
            vol.set_node_kinematic(*i, true)
        }

        vol
    }
}