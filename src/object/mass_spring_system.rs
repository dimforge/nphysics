use std::ops::{AddAssign, SubAssign};
use std::iter;
use std::collections::{HashMap, HashSet};
use std::marker::PhantomData;

use alga::linear::FiniteDimInnerSpace;
use na::{self, Real, DMatrix, DVector, DVectorSlice, DVectorSliceMut, VectorSliceMutN, Cholesky,
         Dynamic, Vector2, Point2, Point3, MatrixN, Unit, CsCholesky, CsMatrix};
use ncollide::utils::{self, DeterministicState};
#[cfg(feature = "dim3")]
use ncollide::procedural::{self, IndexBuffer};
use ncollide::shape::{DeformationsType, Triangle, Polyline, Segment};
#[cfg(feature = "dim3")]
use ncollide::shape::{TriMesh, TetrahedronPointLocation};
use ncollide::query::PointQueryWithLocation;

use counters::Timer;
use object::{Body, BodyPart, BodyHandle, BodyPartHandle, BodyStatus, ActivationStatus};
use solver::{IntegrationParameters, ForceDirection};
use math::{Force, Inertia, Velocity, Vector, Point, Isometry, DIM, Dim};

// FIXME: use an enum for the element itself?
#[derive(Clone)]
pub enum MassSpringElementIndices {
    Triangle(Point3<usize>),
    Segment(Point2<usize>)
}

/// An element of the mass-spring system.
#[derive(Clone)]
pub struct MassSpringElement<N: Real> {
    handle: Option<BodyPartHandle>,
    indices: MassSpringElementIndices,
    phantom: PhantomData<N>
}

#[derive(Copy, Clone, PartialEq, Eq)]
enum SpringConstraintType<N: Real> {
    Equal,
    MinRatio(N),
    MaxRatio(N),
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
        }
    }
}

/// A deformable surface using a mass-spring model with triangular elements.
#[derive(Clone)]
pub struct MassSpringSystem<N: Real> {
    handle: Option<BodyHandle>,
    springs: Vec<Spring<N>>,
    elements: Vec<MassSpringElement<N>>,
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
    pub fn from_trimesh(mesh: &TriMesh<N>, mass: N, stiffness: N, damping_ratio: N) -> Self {
        let ndofs = mesh.points().len() * DIM;
        let mut springs = HashMap::with_hasher(DeterministicState::new());
        let mut elements = Vec::with_capacity(mesh.faces().len());
        let mut positions = DVector::zeros(ndofs);

        for (i, pos) in positions.as_mut_slice().chunks_mut(DIM).enumerate() {
            pos.copy_from_slice(mesh.points()[i].coords.as_slice())
        }

        for face in mesh.faces() {
            let idx = face.indices * DIM;
            let elt = MassSpringElement {
                handle: None,
                indices: MassSpringElementIndices::Triangle(idx),
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
        println!("Number of nodes: {}, of springs: {}", positions.len() / DIM, springs.len());

        MassSpringSystem {
            handle: None,
            springs: springs.values().cloned().collect(),
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
        }
    }

    /// Builds a mass-spring system from a polyline.
    pub fn from_polyline(polyline: &Polyline<N>, mass: N, stiffness: N, damping_ratio: N) -> Self {
        let ndofs = polyline.points().len() * DIM;
        let mut springs = HashMap::with_hasher(DeterministicState::new());
        let mut elements = Vec::with_capacity(polyline.edges().len());
        let mut positions = DVector::zeros(ndofs);

        for (i, pos) in positions.as_mut_slice().chunks_mut(DIM).enumerate() {
            pos.copy_from_slice(polyline.points()[i].coords.as_slice())
        }

        for edge in polyline.edges() {
            let idx = edge.indices * DIM;
            let elt = MassSpringElement {
                handle: None,
                indices: MassSpringElementIndices::Segment(idx),
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
            handle: None,
            springs: springs.values().cloned().collect(),
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
        }
    }

    /// Creates a rectangular quad.
    #[cfg(feature = "dim3")]
    pub fn quad(transform: &Isometry<N>, extents: &Vector2<N>, nx: usize, ny: usize, mass: N, stiffness: N, damping_ratio: N) -> Self {
        let mesh = procedural::quad(extents.x, extents.y, nx, ny);
        let vertices = mesh.coords.iter().map(|pt| transform * pt).collect();
        let indices = mesh.indices.unwrap_unified().into_iter().map(|tri| na::convert(tri)).collect();
        let trimesh = TriMesh::new(vertices, indices, None);
        Self::new(&trimesh, mass, stiffness, damping_ratio)
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

    fn update_augmented_mass_and_forces(&mut self, gravity: &Vector<N>, params: &IntegrationParameters<N>) {
        let mut timer = Timer::new();

        timer.start();
        self.augmented_mass.fill_diagonal(self.node_mass);

        for spring in &mut self.springs {
            let damping = spring.damping_ratio * (spring.stiffness * self.node_mass).sqrt() * na::convert(2.0);
            let v0 = self.velocities.fixed_rows::<Dim>(spring.nodes.0);
            let v1 = self.velocities.fixed_rows::<Dim>(spring.nodes.1);

            let ldot = v1 - v0;
            let l = *spring.dir;

            // Explicit elastic term.
            let coeff = spring.stiffness * (spring.length - spring.rest_length) + damping * ldot.dot(&l);
            let f0 = l * coeff;

            self.accelerations.fixed_rows_mut::<Dim>(spring.nodes.0).add_assign(&f0);
            self.accelerations.fixed_rows_mut::<Dim>(spring.nodes.1).sub_assign(&f0);

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
                self.accelerations.fixed_rows_mut::<Dim>(spring.nodes.0).add_assign(&forward_f0);
                self.accelerations.fixed_rows_mut::<Dim>(spring.nodes.1).sub_assign(&forward_f0);

                // Damping matrix contribution.
                let damping_dt = (l * l.transpose()) * (damping * params.dt);

                // Add to the mass matrix.
                let damping_stiffness = damping_dt + stiffness * (params.dt * params.dt);
                self.augmented_mass.fixed_slice_mut::<Dim, Dim>(spring.nodes.0, spring.nodes.0).add_assign(&damping_stiffness);
                self.augmented_mass.fixed_slice_mut::<Dim, Dim>(spring.nodes.1, spring.nodes.1).add_assign(&damping_stiffness);
                self.augmented_mass.fixed_slice_mut::<Dim, Dim>(spring.nodes.0, spring.nodes.1).sub_assign(&damping_stiffness);
                self.augmented_mass.fixed_slice_mut::<Dim, Dim>(spring.nodes.1, spring.nodes.0).sub_assign(&damping_stiffness);
            }
        }

        /*
         * Add forces due to gravity.
         */
        let gravity_force = gravity * self.node_mass;

        for i in 0..self.positions.len() / DIM {
            let mut acc = self.accelerations.fixed_rows_mut::<Dim>(i * DIM);
            acc += gravity_force
        }
        timer.pause();
//        println!("Assembly time: {}", timer);

        /*
         * Invert the augmented mass.
         */
//        timer.start();
        self.inv_augmented_mass = Cholesky::new(self.augmented_mass.clone()).unwrap();
//        timer.pause();
//        println!("Inversion time: {}", timer);
        self.inv_augmented_mass.solve_mut(&mut self.accelerations);

//        let cs_a: CsMatrix<_, _, _> = self.augmented_mass.clone().into();
//        println!("Shape: {:?}, fill: {}", cs_a.shape(), cs_a.len());
//        let mut chol_cs_a = CsCholesky::new_symbolic(&cs_a);
//        timer.start();
//        let _ = chol_cs_a.decompose_left_looking(cs_a.data.values());
//        timer.pause();
//        println!("Decomposed fill: {}", chol_cs_a.l().unwrap().len());
//        println!("Sparse cholesky time: {}", timer);
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
        self.positions.axpy(params.dt, &self.velocities, N::one());
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
        self.inv_augmented_mass.solve_mut(&mut out);
    }

    fn body_part_jacobian_mul_unit_force(&self, part: &BodyPart<N>, pt: &Point<N>, force_dir: &ForceDirection<N>, out: &mut [N]) {
        // Needed by the non-linear SOR-prox.
        // FIXME: should this `fill` be done by the non-linear SOR-prox itself?
        DVectorSliceMut::from_slice(out, self.ndofs()).fill(N::zero());

        if let ForceDirection::Linear(dir) = force_dir {
            let elt = part.downcast_ref::<MassSpringElement<N>>().expect("The provided body part must be a triangular mass-spring element");

            match elt.indices {
                MassSpringElementIndices::Triangle(indices) => {
                    let a = self.positions.fixed_rows::<Dim>(indices.x).into_owned();
                    let b = self.positions.fixed_rows::<Dim>(indices.y).into_owned();
                    let c = self.positions.fixed_rows::<Dim>(indices.z).into_owned();

                    let tri = Triangle::new(
                        Point::from_coordinates(a),
                        Point::from_coordinates(b),
                        Point::from_coordinates(c),
                    );

                    // XXX: This is extremely costly!
                    let proj = tri.project_point_with_location(&Isometry::identity(), pt, false).1;
                    let bcoords = proj.barycentric_coordinates().unwrap();

                    VectorSliceMutN::<N, Dim>::from_slice(&mut out[indices.x..]).copy_from(&(**dir * bcoords[0]));
                    VectorSliceMutN::<N, Dim>::from_slice(&mut out[indices.y..]).copy_from(&(**dir * bcoords[1]));
                    VectorSliceMutN::<N, Dim>::from_slice(&mut out[indices.z..]).copy_from(&(**dir * bcoords[2]));
                }
                MassSpringElementIndices::Segment(indices) => {
                    let a = self.positions.fixed_rows::<Dim>(indices.x).into_owned();
                    let b = self.positions.fixed_rows::<Dim>(indices.y).into_owned();

                    let seg = Segment::new(
                        Point::from_coordinates(a),
                        Point::from_coordinates(b),
                    );

                    // XXX: This is extremely costly!
                    let proj = seg.project_point_with_location(&Isometry::identity(), pt, false).1;
                    let bcoords = proj.barycentric_coordinates();

                    VectorSliceMutN::<N, Dim>::from_slice(&mut out[indices.x..]).copy_from(&(**dir * bcoords[0]));
                    VectorSliceMutN::<N, Dim>::from_slice(&mut out[indices.y..]).copy_from(&(**dir * bcoords[1]));
                }
            }
        }
    }

    fn body_part_point_velocity(&self, part: &BodyPart<N>, point: &Point<N>, force_dir: &ForceDirection<N>) -> N {
        unimplemented!()
    }

    #[inline]
    fn has_active_internal_constraints(&mut self) -> bool {
        false
    }

    #[inline]
    fn setup_internal_velocity_constraints(&mut self, dvels: &mut DVectorSliceMut<N>) {}

    #[inline]
    fn step_solve_internal_velocity_constraints(&mut self, dvels: &mut DVectorSliceMut<N>) {}

    #[inline]
    fn step_solve_internal_position_constraints(&mut self, params: &IntegrationParameters<N>) {}
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