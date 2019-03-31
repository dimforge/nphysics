use std::ops::AddAssign;
use std::iter;
use std::collections::HashMap;
use std::sync::Arc;
use std::any::Any;
use either::Either;

use na::{self, RealField, Point2, Point3, Vector3, Matrix2, Matrix2x3, DMatrix,
         DVector, DVectorSlice, DVectorSliceMut, Cholesky, Dynamic, Vector2, Unit};
use ncollide::utils::{self, DeterministicState};
use ncollide::shape::{Polyline, DeformationsType, ShapeHandle};

use crate::object::{Body, BodyPart, BodyHandle, BodyPartHandle, BodyStatus, ActivationStatus,
                    FiniteElementIndices, DeformableColliderDesc, BodyDesc, BodyUpdateStatus};
use crate::solver::{IntegrationParameters, ForceDirection};
use crate::math::{Force, ForceType, Inertia, Velocity, Matrix, Dim, DIM, Point, Isometry,
                  SpatialVector, RotationMatrix, Vector, Translation};
use crate::object::fem_helper;
use crate::world::{World, ColliderWorld};
use crate::utils::{UserData, UserDataBox};

/// One element of a deformable surface.
#[derive(Clone)]
pub struct TriangularElement<N: RealField> {
    handle: BodyPartHandle,
    indices: Point3<usize>,
    com: Point<N>,
    rot: RotationMatrix<N>,
    inv_rot: RotationMatrix<N>,
    j: Matrix<N>,
    local_j_inv: Matrix2x3<N>,
    total_strain: SpatialVector<N>,
    plastic_strain: SpatialVector<N>,
    surface: N,
    density: N,
}

/// A deformable surface using FEM to simulate linear elasticity.
///
/// The surface is described by a set of triangle elements. This
/// implements an isoparametric approach where the interpolations are linear.
pub struct FEMSurface<N: RealField> {
    name: String,
    handle: BodyHandle,
    elements: Vec<TriangularElement<N>>,
    kinematic_nodes: DVector<bool>,
    positions: DVector<N>,
    velocities: DVector<N>,
    accelerations: DVector<N>,
    forces: DVector<N>,
    augmented_mass: DMatrix<N>,
    inv_augmented_mass: Cholesky<N, Dynamic>,

    workspace: DVector<N>,

    // Parameters
    gravity_enabled: bool,
    rest_positions: DVector<N>,
    damping_coeffs: (N, N),
    young_modulus: N,
    poisson_ratio: N,
    plasticity_threshold: N,
    plasticity_creep: N,
    plasticity_max_force: N,
    // Elasticity coefficients computed from the young modulus
    // and poisson ratio.
    d0: N,
    d1: N,
    d2: N,

    companion_id: usize,
    activation: ActivationStatus<N>,
    status: BodyStatus,
    update_status: BodyUpdateStatus,


    user_data: Option<Box<Any + Send + Sync>>,
}

impl<N: RealField> FEMSurface<N> {
    /// Initializes a new deformable surface from its triangle elements.
    fn new(handle: BodyHandle, vertices: &[Point<N>], triangles: &[Point3<usize>], pos: &Isometry<N>,
           scale: &Vector<N>, density: N, young_modulus: N, poisson_ratio: N, damping_coeffs: (N, N)) -> Self {
        let ndofs = vertices.len() * DIM;
        let mut rest_positions = DVector::zeros(ndofs);

        for (i, pt)  in vertices.iter().enumerate() {
            let pt = pos * Point::from(pt.coords.component_mul(&scale));
            rest_positions.fixed_rows_mut::<Dim>(i * DIM).copy_from(&pt.coords);
        }

        let elements = triangles.iter().enumerate().map(|(i, idx)| {
            let rest_a = rest_positions.fixed_rows::<Dim>(idx.x * 2);
            let rest_b = rest_positions.fixed_rows::<Dim>(idx.y * 2);
            let rest_c = rest_positions.fixed_rows::<Dim>(idx.z * 2);

            let rest_ab = rest_b - rest_a;
            let rest_ac = rest_c - rest_a;

            let local_j = Matrix2::new(
                rest_ab.x, rest_ab.y,
                rest_ac.x, rest_ac.y,
            );

            let local_j_inv = local_j.try_inverse().unwrap_or(Matrix2::identity());
            let local_j_inv = Matrix2x3::new(
                -local_j_inv.m11 - local_j_inv.m12, local_j_inv.m11, local_j_inv.m12,
                -local_j_inv.m21 - local_j_inv.m22, local_j_inv.m21, local_j_inv.m22,
            );

            TriangularElement {
                handle: BodyPartHandle(handle, i),
                indices: idx * DIM,
                com: Point::origin(),
                rot: RotationMatrix::identity(),
                inv_rot: RotationMatrix::identity(),
                j: local_j,
                local_j_inv,
                total_strain: SpatialVector::zeros(),
                plastic_strain: SpatialVector::zeros(),
                surface: local_j.determinant() / na::convert(2.0),
                density,
            }
        }).collect();

        let (d0, d1, d2) = fem_helper::elasticity_coefficients(young_modulus, poisson_ratio);

        FEMSurface {
            name: String::new(),
            handle,
            elements,
            kinematic_nodes: DVector::repeat(vertices.len(), false),
            positions: rest_positions.clone(),
            velocities: DVector::zeros(ndofs),
            accelerations: DVector::zeros(ndofs),
            forces: DVector::zeros(ndofs),
            augmented_mass: DMatrix::zeros(ndofs, ndofs),
            inv_augmented_mass: Cholesky::new(DMatrix::zeros(0, 0)).unwrap(),
            workspace: DVector::zeros(ndofs),
            rest_positions,
            damping_coeffs,
            young_modulus,
            poisson_ratio,
            companion_id: 0,
            plasticity_threshold: N::zero(),
            plasticity_max_force: N::zero(),
            plasticity_creep: N::zero(),
            gravity_enabled: true,
            d0, d1, d2,
            activation: ActivationStatus::new_active(),
            status: BodyStatus::Dynamic,
            update_status: BodyUpdateStatus::all(),
            user_data: None
        }
    }

    user_data_accessors!();

    /// The position of this body in generalized coordinates.
    #[inline]
    pub fn positions(&self) -> &DVector<N> {
        &self.positions
    }

    /// The position of this body in generalized coordinates.
    #[inline]
    pub fn positions_mut(&mut self) -> &mut DVector<N> {
        &mut self.positions
    }

    /// The velocity of this body in generalized coordinates.
    #[inline]
    pub fn velocities(&self) -> &DVector<N> {
        &self.velocities
    }

    /// The mutable velocity of this body in generalized coordinates.
    #[inline]
    pub fn velocities_mut(&mut self) -> &mut DVector<N> {
        self.update_status.set_velocity_changed(true);
        &mut self.velocities
    }


    /// Sets the plastic properties of this deformable surface.
    ///
    /// Note that large plasticity creep coefficient can yield to significant instability.
    pub fn set_plasticity(&mut self, strain_threshold: N, creep: N, max_force: N) {
        self.plasticity_threshold = strain_threshold;
        self.plasticity_creep = creep;
        self.plasticity_max_force = max_force;
    }

    /// Sets the young modulus of this deformable surface.
    pub fn set_young_modulus(&mut self, young_modulus: N) {
        self.update_status.set_local_inertia_changed(true);
        self.young_modulus = young_modulus;

        let (d0, d1, d2) = fem_helper::elasticity_coefficients(self.young_modulus, self.poisson_ratio);
        self.d0 = d0;
        self.d1 = d1;
        self.d2 = d2;
    }

    /// Sets the poisson ratio of this deformable surface.
    pub fn set_poisson_ratio(&mut self, poisson_ratio: N) {
        self.update_status.set_local_inertia_changed(true);
        self.poisson_ratio = poisson_ratio;

        let (d0, d1, d2) = fem_helper::elasticity_coefficients(self.young_modulus, self.poisson_ratio);
        self.d0 = d0;
        self.d1 = d1;
        self.d2 = d2;
    }

    /// The handle of this body.
    pub fn handle(&self) -> BodyHandle {
        self.handle
    }

    fn assemble_mass_with_damping(&mut self, dt: N) {
        let mass_damping = dt * self.damping_coeffs.0;

        for elt in self.elements.iter() {
            let coeff_mass = elt.density * elt.surface / na::convert::<_, N>(12.0f64) * (N::one() + mass_damping);

            for a in 0..3 {
                let ia = elt.indices[a];

                if !self.kinematic_nodes[ia / DIM] {
                    for b in 0..3 {
                        let ib = elt.indices[b];

                        if !self.kinematic_nodes[ib / DIM] {
                            let mass_contribution = if a == b {
                                coeff_mass * na::convert(2.0)
                            } else {
                                coeff_mass
                            };

                            let mut node_mass = self.augmented_mass.fixed_slice_mut::<Dim, Dim>(ia, ib);
                            for i in 0..DIM {
                                node_mass[(i, i)] += mass_contribution;
                            }
                        }
                    }
                }
            }
        }

        // Set the identity for kinematic nodes.
        for i in 0..self.kinematic_nodes.len() {
            if self.kinematic_nodes[i] {
                self.augmented_mass.fixed_slice_mut::<Dim, Dim>(i * DIM, i * DIM).fill_diagonal(N::one());
            }
        }
    }


    fn assemble_stiffness(&mut self, dt: N) {
        let _1: N = na::one();
        let _2: N = na::convert(2.0);
        let stiffness_coeff = dt * (dt + self.damping_coeffs.1);

        for elt in self.elements.iter_mut() {
            let d0_surf = self.d0 * elt.surface;
            let d1_surf = self.d1 * elt.surface;
            let d2_surf = self.d2 * elt.surface;

            for a in 0..3 {
                let ia = elt.indices[a];

                if !self.kinematic_nodes[ia / DIM] {
                    let bn = elt.local_j_inv[(0, a)];
                    let cn = elt.local_j_inv[(1, a)];

                    // Fields of P_n * elt.volume:
                    //
                    // let P_n = Matrix2x3::new(
                    //   bn * d0, bn * d1, cn * d2,
                    //   cn * d1, cn * d0, bn * d2,
                    // ) * elt.surface;
                    let bn0 = bn * d0_surf;
                    let bn1 = bn * d1_surf;
                    let bn2 = bn * d2_surf;
                    let cn0 = cn * d0_surf;
                    let cn1 = cn * d1_surf;
                    let cn2 = cn * d2_surf;

                    /*
                     * Add elastic strain.
                     */
                    for b in 0..3 {
                        let bm = elt.local_j_inv[(0, b)];
                        let cm = elt.local_j_inv[(1, b)];


                        // NOTE: this could be precomputed as this is constant.
                        // however we don't because this has a significant memory
                        // cost (Matrix2 * 3 * 3 for each element).
                        let node_stiffness = Matrix2::new(
                            bn0 * bm + cn2 * cm, bn1 * cm + cn2 * bm,
                            cn1 * bm + bn2 * cm, cn0 * cm + bn2 * bm,
                        );

                        let rot_stiffness = elt.rot * node_stiffness;
                        let ib = elt.indices[b];

                        if !self.kinematic_nodes[ib / DIM] {
                            let mut mass_part = self.augmented_mass.fixed_slice_mut::<Dim, Dim>(ia, ib);
                            mass_part.gemm(stiffness_coeff, &rot_stiffness, elt.inv_rot.matrix(), N::one());
                        }
                    }
                }
            }
        }
    }

    fn assemble_forces(&mut self, gravity: &Vector<N>, params: &IntegrationParameters<N>) {
        let _1: N = na::one();
        let _2: N = na::convert(2.0);
        let dt = params.dt;

        self.accelerations.copy_from(&self.forces);

        // Gravity
        if self.gravity_enabled {
            for elt in self.elements.iter() {
                let contribution = gravity * (elt.density * elt.surface * na::convert::<_, N>(1.0 / 3.0));

                for k in 0..3 {
                    let ie = elt.indices[k];

                    if !self.kinematic_nodes[ie / DIM] {
                        let mut forces_part = self.accelerations.fixed_rows_mut::<Dim>(ie);
                        forces_part += contribution;
                    }
                }
            }
        }

        for elt in self.elements.iter_mut() {

            let d0_surf = self.d0 * elt.surface;
            let d1_surf = self.d1 * elt.surface;
            let d2_surf = self.d2 * elt.surface;

            /*
             *
             * Plastic strain.
             *
             */
            elt.total_strain = Vector3::zeros();

            // Compute plastic strain.
            for a in 0..3 {
                let bn = elt.local_j_inv[(0, a)];
                let cn = elt.local_j_inv[(1, a)];

                let ia = elt.indices[a];
                let vel_part = self.velocities.fixed_rows::<Dim>(ia);
                let pos_part = self.positions.fixed_rows::<Dim>(ia);
                let ref_pos_part = self.rest_positions.fixed_rows::<Dim>(ia);
                let dpos = elt.inv_rot * (vel_part * dt + pos_part) - ref_pos_part;
                // total_strain += B_n * dpos
                elt.total_strain += Vector3::new(
                    bn * dpos.x,
                    cn * dpos.y,
                    cn * dpos.x + bn * dpos.y,
                );
            }

            let strain = elt.total_strain - elt.plastic_strain;
            if strain.norm() > self.plasticity_threshold {
                let coeff = params.dt * (N::one() / params.dt).min(self.plasticity_creep);
                elt.plastic_strain += strain * coeff;
            }

            if let Some((dir, magnitude)) = Unit::try_new_and_get(elt.plastic_strain, N::zero()) {
                if magnitude > self.plasticity_max_force {
                    elt.plastic_strain = *dir * self.plasticity_max_force;
                }
            }

            for a in 0..3 {
                let ia = elt.indices[a];

                if !self.kinematic_nodes[ia / DIM] {
                    let bn = elt.local_j_inv[(0, a)];
                    let cn = elt.local_j_inv[(1, a)];

                    // Fields of P_n * elt.volume:
                    //
                    // let P_n = Matrix2x3::new(
                    //   bn * d0, bn * d1, cn * d2,
                    //   cn * d1, cn * d0, bn * d2,
                    // ) * elt.surface;
                    let bn0 = bn * d0_surf;
                    let bn1 = bn * d1_surf;
                    let bn2 = bn * d2_surf;
                    let cn0 = cn * d0_surf;
                    let cn1 = cn * d1_surf;
                    let cn2 = cn * d2_surf;

                    /*
                     * Add plastic strain.
                     */
                    // P_n * strain
                    let strain = elt.total_strain - elt.plastic_strain;
                    #[cfg_attr(rustfmt, rustfmt_skip)]
                        let projected_strain = Vector2::new(
                        bn0 * strain.x + bn1 * strain.y + cn2 * strain.z,
                        cn1 * strain.x + cn0 * strain.y + bn2 * strain.z,
                    );

                    let mut force_part = self.accelerations.fixed_rows_mut::<Dim>(ia);
                    force_part -= elt.rot * projected_strain;
                }
            }
        }
    }

    /// Returns the triangles at the boundary of this surface.
    ///
    /// Each element of the returned `Vec` is a tuple containing the 3 indices of the triangle
    /// vertices, and the index of the corresponding triangle element.
    #[cfg(feature = "dim2")]
    pub fn boundary(&self) -> Vec<(Point2<usize>, usize)> {
        fn key(a: usize, b: usize) -> (usize, usize) {
            let (sa, sb) = utils::sort2(&a, &b);
            (*sa, *sb)
        }

        let mut faces = HashMap::with_hasher(DeterministicState::new());

        for (i, elt) in self.elements.iter().enumerate() {
            let k1 = key(elt.indices.x, elt.indices.y);
            let k2 = key(elt.indices.y, elt.indices.z);
            let k3 = key(elt.indices.z, elt.indices.x);

            faces.entry(k1).or_insert((0, elt.indices.z, i)).0.add_assign(1);
            faces.entry(k2).or_insert((0, elt.indices.x, i)).0.add_assign(1);
            faces.entry(k3).or_insert((0, elt.indices.y, i)).0.add_assign(1);
        }

        let boundary = faces.iter().filter_map(|(k, n)| {
            if n.0 == 1 {
                // Ensure the triangle has an outward normal.
                // FIXME: there is a much more efficient way of doing this, given the
                // triangles orientations and the face.
                let a = self.positions.fixed_rows::<Dim>(k.0);
                let b = self.positions.fixed_rows::<Dim>(k.1);
                let c = self.positions.fixed_rows::<Dim>(n.1);

                let ab = b - a;
                let ac = c - a;

                if ab.perp(&ac) < N::zero() {
                    Some((Point2::new(k.0, k.1), n.2))
                } else {
                    Some((Point2::new(k.1, k.0), n.2))
                }
            } else {
                None
            }
        }).collect();

        boundary
    }

    /// Returns a triangle mesh at the boundary of this surface as well as a mapping between the mesh
    /// vertices and this surface degrees of freedom and the mapping between the mesh triangles and
    /// this surface body parts (the triangle elements).
    ///
    /// The output is (triangle mesh, deformation indices, element to body part map).
    #[cfg(feature = "dim2")]
    pub fn boundary_polyline(&self) -> (Polyline<N>, Vec<usize>, Vec<usize>) {
        const INVALID: usize = usize::max_value();
        let mut deformation_indices = Vec::new();
        let mut indices = self.boundary();
        let mut idx_remap: Vec<usize> = iter::repeat(INVALID).take(self.positions.len() / 2).collect();
        let mut vertices = Vec::new();

        for (idx, _) in &mut indices {
            for i in 0..2 {
                let idx_i = &mut idx[i];
                if idx_remap[*idx_i / 2] == INVALID {
                    let new_id = vertices.len();
                    vertices.push(Point2::new(
                        self.positions[*idx_i + 0],
                        self.positions[*idx_i + 1])
                    );
                    deformation_indices.push(*idx_i);
                    idx_remap[*idx_i / 2] = new_id;
                    *idx_i = new_id;
                } else {
                    *idx_i = idx_remap[*idx_i / 2];
                }
            }
        }

        let body_parts = indices.iter().map(|i| i.1).collect();
        let indices = indices.into_iter().map(|i| i.0).collect();

        (Polyline::new(vertices, Some(indices)), deformation_indices, body_parts)
    }

    /// Renumber degrees of freedom so that the `deformation_indices[i]`-th DOF becomes the `i`-th one.
    pub fn renumber_dofs(&mut self, deformation_indices: &[usize]) {
        let mut dof_map: Vec<_> = (0..).take(self.positions.len()).collect();
        let mut remapped: Vec<_> = iter::repeat(false).take(self.positions.len()).collect();
        let mut new_positions = DVector::zeros(self.positions.len());
        let mut new_rest_positions = DVector::zeros(self.positions.len());

        for (target_i, orig_i) in deformation_indices.iter().cloned().enumerate() {
            assert!(!remapped[orig_i], "Duplicate DOF remapping found.");
            let target_i = target_i * 2;
            new_positions.fixed_rows_mut::<Dim>(target_i).copy_from(&self.positions.fixed_rows::<Dim>(orig_i));
            new_rest_positions.fixed_rows_mut::<Dim>(target_i).copy_from(&self.rest_positions.fixed_rows::<Dim>(orig_i));
            dof_map[orig_i] = target_i;
            remapped[orig_i] = true;
        }

        let mut curr_target = deformation_indices.len() * 2;

        for orig_i in (0..self.positions.len()).step_by(2) {
            if !remapped[orig_i] {
                new_positions.fixed_rows_mut::<Dim>(curr_target).copy_from(&self.positions.fixed_rows::<Dim>(orig_i));
                new_rest_positions.fixed_rows_mut::<Dim>(curr_target).copy_from(&self.rest_positions.fixed_rows::<Dim>(orig_i));
                dof_map[orig_i] = curr_target;
                curr_target += 2;
            }
        }

        for elt in &mut self.elements {
            elt.indices.coords.apply(|i| dof_map[i]);
        }

        self.positions = new_positions;
        self.rest_positions = new_rest_positions;
    }

// FIXME: add a method to apply a transformation to the whole surface.

    /// Constructs an axis-aligned cube with regular subdivisions along each axis.
    ///
    /// The cube is subdivided `nx` (resp. `ny`) times along
    /// the `x` (resp. `y`) axis.
    fn quad(handle: BodyHandle, pos: &Isometry<N>, extents: &Vector2<N>, nx: usize, ny: usize, density: N, young_modulus: N, poisson_ratio: N, damping_coeffs: (N, N)) -> Self {
        let mut vertices = Vec::new();
        let mut indices = Vec::new();

        // First, generate the vertices.
        let x_step: N = na::convert(1.0 / nx as f64);
        let y_step: N = na::convert(1.0 / ny as f64);

        for i in 0..=nx {
            let x = x_step * na::convert(i as f64) - na::convert(0.5);

            for j in 0..=ny {
                let y = y_step * na::convert(j as f64) - na::convert(0.5);
                let mut coords = Vector::zeros();
                coords.x = x;
                coords.y = y;
                vertices.push(Point::from(coords))
            }
        }

        // Second, generate indices.
        for i in 0..nx {
            for j in 0..ny {
                //         2 o----------o 3
                //           |          |                y
                //           |          |                ^
                //           |          |                |
                //           o----------o                 --> x
                //           0          1
                /* Local quad indices:
                let a = Point3::new(0, 1, 2);
                let b = Point3::new(1, 3, 2);
                */
                fn shift(ny: usize, di: usize, dj: usize) -> usize {
                    di * (ny + 1) + dj
                }
                // _0 = node at (i, j)
                let _0 = i * (ny + 1) + j;
                let _1 = _0 + shift(ny, 1, 0);
                let _2 = _0 + shift(ny, 0, 1);
                let _3 = _0 + shift(ny, 1, 1);

                indices.push(Point3::new(_0, _1, _2));
                indices.push(Point3::new(_1, _3, _2));
            }
        }

        Self::new(handle, &vertices, &indices, pos, extents, density, young_modulus, poisson_ratio, damping_coeffs)
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
}

impl<N: RealField> Body<N> for FEMSurface<N> {
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

    #[inline]
    fn deformed_positions(&self) -> Option<(DeformationsType, &[N])> {
        Some((DeformationsType::Vectors, self.positions.as_slice()))
    }

    #[inline]
    fn deformed_positions_mut(&mut self) -> Option<(DeformationsType, &mut [N])> {
        self.update_status.set_local_inertia_changed(true);
        Some((DeformationsType::Vectors, self.positions.as_mut_slice()))
    }

    fn update_kinematics(&mut self) {
        if !self.update_status.position_changed() {
            return;
        }

        for elt in &mut self.elements {
            let a = self.positions.fixed_rows::<Dim>(elt.indices.x);
            let b = self.positions.fixed_rows::<Dim>(elt.indices.y);
            let c = self.positions.fixed_rows::<Dim>(elt.indices.z);

            let ab = b - a;
            let ac = c - a;

            elt.j = Matrix::new(
                ab.x, ab.y,
                ac.x, ac.y,
            );

            let g = (elt.local_j_inv.fixed_slice::<Dim, Dim>(0, 1) * elt.j).transpose();
            elt.rot = RotationMatrix::from_matrix_eps(&g, N::default_epsilon(), 20, elt.rot);
            elt.inv_rot = elt.rot.inverse();
            elt.com = Point::from(a + b + c) * na::convert::<_, N>(1.0 / 3.0);
        }
    }

    /// Update the dynamics property of this deformable surface.
    fn update_dynamics(&mut self, dt: N) {
        if self.update_status.inertia_needs_update() && self.status == BodyStatus::Dynamic {
            if !self.is_active() {
                self.activate();
            }

            self.augmented_mass.fill(N::zero());
            self.assemble_mass_with_damping(dt);
            self.assemble_stiffness(dt);

            // FIXME: avoid allocation inside Cholesky at each timestep.
            // FIXME: if Cholesky fails fallback to some sort of mass-spring formulation?
            //        If we do so we should add a bool to let give the user the ability to check which
            //        model has been used during the last timestep.
            self.inv_augmented_mass = Cholesky::new(self.augmented_mass.clone()).expect("Singular system found.");
        }
    }

    /// Update the dynamics property of this deformable surface.
    fn update_acceleration(&mut self,
                           gravity: &Vector<N>,
                           params: &IntegrationParameters<N>) {
        self.assemble_forces(gravity, params);
        self.inv_augmented_mass.solve_mut(&mut self.accelerations);
    }

    fn clear_forces(&mut self) {
        self.forces.fill(N::zero())
    }

    fn clear_update_flags(&mut self) {
        self.update_status.clear()
    }

    fn update_status(&self) -> BodyUpdateStatus {
        self.update_status
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

    fn activation_status(&self) -> &ActivationStatus<N> {
        &self.activation
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
        let ndofs = self.velocities.len();
        DVectorSliceMut::from_slice(self.velocities.as_mut_slice(), ndofs)
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

    fn set_deactivation_threshold(&mut self, threshold: Option<N>) {
        self.activation.set_deactivation_threshold(threshold)
    }

    fn part(&self, id: usize) -> Option<&BodyPart<N>> {
        self.elements.get(id).map(|b| b as &BodyPart<N>)
    }

    fn world_point_at_material_point(&self, part: &BodyPart<N>, point: &Point<N>) -> Point<N> {
        let elt = part.downcast_ref::<TriangularElement<N>>().expect("The provided body part must be a triangular element");
        fem_helper::world_point_at_material_point(FiniteElementIndices::Triangle(elt.indices), &self.positions, point)
    }

    fn position_at_material_point(&self, part: &BodyPart<N>, point: &Point<N>) -> Isometry<N> {
        let elt = part.downcast_ref::<TriangularElement<N>>().expect("The provided body part must be a triangular element");
        let pt = fem_helper::world_point_at_material_point(FiniteElementIndices::Triangle(elt.indices), &self.positions, point);
        Isometry::from_parts(Translation::from(pt.coords), na::one())
    }

    fn material_point_at_world_point(&self, part: &BodyPart<N>, point: &Point<N>) -> Point<N> {
        let elt = part.downcast_ref::<TriangularElement<N>>().expect("The provided body part must be a triangular element");
        fem_helper::material_point_at_world_point(FiniteElementIndices::Triangle(elt.indices), &self.positions, point)
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
        let elt = part.downcast_ref::<TriangularElement<N>>().expect("The provided body part must be a triangular element");
        fem_helper::fill_contact_geometry_fem(
            self.ndofs(),
            self.status,
            FiniteElementIndices::Triangle(elt.indices),
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
    fn setup_internal_velocity_constraints(&mut self, _: &DVectorSlice<N>, _: &IntegrationParameters<N>) {}

    #[inline]
    fn warmstart_internal_velocity_constraints(&mut self, _: &mut DVectorSliceMut<N>) {}

    #[inline]
    fn step_solve_internal_velocity_constraints(&mut self, _: &mut DVectorSliceMut<N>) {}

    #[inline]
    fn step_solve_internal_position_constraints(&mut self, _: &IntegrationParameters<N>) {}

    fn apply_force_at_local_point(&mut self, part_id: usize, force: &Vector<N>, point: &Point<N>, force_type: ForceType, auto_wake_up: bool) {
        if self.status != BodyStatus::Dynamic {
            return;
        }

        if auto_wake_up {
            self.activate()
        }

        let element = &self.elements[part_id];
        let forces = [
            force * (N::one() - point.x - point.y),
            force * point.x,
            force * point.y,
        ];

        match force_type {
            ForceType::Force => {
                for i in 0..3 {
                    if !self.kinematic_nodes[element.indices[i] / DIM] {
                        self.forces.fixed_rows_mut::<Dim>(element.indices[i]).add_assign(forces[i]);
                    }
                }
            }
            ForceType::Impulse => {
                let dvel = &mut self.workspace;
                dvel.fill(N::zero());
                for i in 0..3 {
                    if !self.kinematic_nodes[element.indices[i] / DIM] {
                        dvel.fixed_rows_mut::<Dim>(element.indices[i]).copy_from(&forces[i]);
                    }
                }
                self.inv_augmented_mass.solve_mut(dvel);
                self.velocities += &*dvel;
            }
            ForceType::AccelerationChange => {
                let mass = element.density * element.surface;

                for i in 0..3 {
                    if !self.kinematic_nodes[element.indices[i] / DIM] {
                        self.forces.fixed_rows_mut::<Dim>(element.indices[i]).add_assign(forces[i] * mass);
                    }
                }
            }
            ForceType::VelocityChange => {
                for i in 0..3 {
                    if !self.kinematic_nodes[element.indices[i] / DIM] {
                        self.velocities.fixed_rows_mut::<Dim>(element.indices[i]).add_assign(forces[i]);
                    }
                }
            }
        }
    }

    fn apply_force(&mut self, part_id: usize, force: &Force<N>, force_type: ForceType, auto_wake_up: bool) {
        let _1_3: N = na::convert(1.0 / 3.0);
        let barycenter = Point::new(_1_3, _1_3);
        self.apply_force_at_local_point(part_id, &force.linear, &barycenter, force_type, auto_wake_up)
    }

    fn apply_local_force(&mut self, part_id: usize, force: &Force<N>, force_type: ForceType, auto_wake_up: bool) {
        let world_force = Force::new(self.elements[part_id].rot * force.linear, force.angular);
        self.apply_force(part_id, &world_force, force_type, auto_wake_up);
    }

    fn apply_force_at_point(&mut self, part_id: usize, force: &Vector<N>, point: &Point<N>, force_type: ForceType, auto_wake_up: bool) {
        let local_point = self.material_point_at_world_point(&self.elements[part_id], point);
        self.apply_force_at_local_point(part_id, &force, &local_point, force_type, auto_wake_up)
    }

    fn apply_local_force_at_point(&mut self, part_id: usize, force: &Vector<N>, point: &Point<N>, force_type: ForceType, auto_wake_up: bool) {
        let world_force = self.elements[part_id].rot * force;
        let local_point = self.material_point_at_world_point(&self.elements[part_id], point);
        self.apply_force_at_local_point(part_id, &world_force, &local_point, force_type, auto_wake_up);
    }


    fn apply_local_force_at_local_point(&mut self, part_id: usize, force: &Vector<N>, point: &Point<N>, force_type: ForceType, auto_wake_up: bool) {
        let world_force = self.elements[part_id].rot * force;
        self.apply_force_at_local_point(part_id, &world_force, &point, force_type, auto_wake_up);
    }
}


impl<N: RealField> BodyPart<N> for TriangularElement<N> {
    fn part_handle(&self) -> BodyPartHandle {
        self.handle
    }

    fn center_of_mass(&self) -> Point<N> {
        self.com
    }

    fn position(&self) -> Isometry<N> {
        Isometry::from_parts(self.com.coords.into(), self.rot.into())
    }

    fn velocity(&self) -> Velocity<N> {
        unimplemented!()
    }

    fn inertia(&self) -> Inertia<N> {
        Inertia::new(self.surface * self.density, N::one())
    }

    fn local_inertia(&self) -> Inertia<N> {
        Inertia::new(self.surface * self.density, N::one())
    }
}


enum FEMSurfaceDescGeometry<'a, N: RealField> {
    Quad(usize, usize),
    Triangles(&'a [Point<N>], &'a [Point3<usize>])
}

/// A builder for FEMSurface bodies.
pub struct FEMSurfaceDesc<'a, N: RealField> {
    name: String,
    user_data: Option<UserDataBox>,
    geom: FEMSurfaceDescGeometry<'a, N>,
    scale: Vector<N>,
    position: Isometry<N>,
    young_modulus: N,
    poisson_ratio: N,
    sleep_threshold: Option<N>,
    collider_enabled: bool,
    mass_damping: N,
    stiffness_damping: N,
    density: N,
    plasticity: (N, N, N),
    kinematic_nodes: Vec<usize>,
    status: BodyStatus,
    gravity_enabled: bool,
}

impl<'a, N: RealField> FEMSurfaceDesc<'a, N> {
    fn with_geometry(geom: FEMSurfaceDescGeometry<'a, N>) -> Self {
        FEMSurfaceDesc {
            name: String::new(),
            user_data: None,
            gravity_enabled: true,
            geom,
            scale: Vector::repeat(N::one()),
            position: Isometry::identity(),
            young_modulus: na::convert(1.0e3),
            poisson_ratio: N::zero(),
            sleep_threshold: Some(ActivationStatus::default_threshold()),
            collider_enabled: false,
            mass_damping: na::convert(0.2),
            stiffness_damping: N::zero(),
            density: N::one(),
            plasticity: (N::zero(), N::zero(), N::zero()),
            kinematic_nodes: Vec::new(),
            status: BodyStatus::Dynamic
        }
    }

    /// Create a surface form the given tiangles.
    pub fn new(vertices: &'a [Point<N>], triangles: &'a [Point3<usize>]) -> Self {
        Self::with_geometry(FEMSurfaceDescGeometry::Triangles(vertices, triangles))
    }

    /// Create a surface form the given triangles.
    pub fn quad(subdiv_x: usize, subdiv_y: usize) -> Self {
        Self::with_geometry(FEMSurfaceDescGeometry::Quad(subdiv_x, subdiv_y))
    }

    /// Mark all nodes as non-kinematic.
    pub fn clear_kinematic_nodes(&mut self) -> &mut Self {
        self.kinematic_nodes.clear();
        self
    }

    user_data_desc_accessors!();

    desc_custom_setters!(
        self.collider_enabled, set_collider_enabled, enable: bool | { self.collider_enabled = enable }
        self.plasticity, set_plasticity, strain_threshold: N, creep: N, max_force: N | { self.plasticity = (strain_threshold, creep, max_force) }
        self.kinematic_nodes, set_nodes_kinematic, nodes: &[usize] | { self.kinematic_nodes.extend_from_slice(nodes) }
        self.translation, set_translation, vector: Vector<N> | { self.position.translation.vector = vector }
    );

    desc_setters!(
        gravity_enabled, enable_gravity, gravity_enabled: bool
        scale, set_scale, scale: Vector<N>
        young_modulus, set_young_modulus, young_modulus: N
        poisson_ratio, set_poisson_ratio, poisson_ratio: N
        sleep_threshold, set_sleep_threshold, sleep_threshold: Option<N>
        mass_damping, set_mass_damping, mass_damping: N
        stiffness_damping, set_stiffness_damping, stiffness_damping: N
        density, set_density, density: N
        status, set_status, status: BodyStatus
        position, set_position, position: Isometry<N>
        name, set_name, name: String
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
        [val] get_young_modulus -> young_modulus: N
        [val] get_poisson_ratio -> poisson_ratio: N
        [val] get_sleep_threshold -> sleep_threshold: Option<N>
        [val] get_mass_damping -> mass_damping: N
        [val] get_stiffness_damping -> stiffness_damping: N
        [val] get_density -> density: N
        [val] get_status -> status: BodyStatus
        [val] is_collider_enabled -> collider_enabled: bool
        [ref] get_position -> position: Isometry<N>
        [ref] get_scale -> scale: Vector<N>
    );

    /// Build a deformable surface.
    pub fn build<'w>(&self, world: &'w mut World<N>) -> &'w mut FEMSurface<N> {
        world.add_body(self)
    }
}

impl<'a, N: RealField> BodyDesc<N> for FEMSurfaceDesc<'a, N> {
    type Body = FEMSurface<N>;

    fn build_with_handle(&self, cworld: &mut ColliderWorld<N>, handle: BodyHandle) -> FEMSurface<N> {
        let mut vol = match self.geom {
            FEMSurfaceDescGeometry::Quad(nx, ny) =>
                FEMSurface::quad(handle, &self.position, &self.scale,
                                 nx, ny, self.density, self.young_modulus,
                                 self.poisson_ratio,
                                 (self.mass_damping, self.stiffness_damping)),
            FEMSurfaceDescGeometry::Triangles(pts, idx) =>
                FEMSurface::new(handle, pts, idx, &self.position, &self.scale,
                                self.density, self.young_modulus, self.poisson_ratio,
                                (self.mass_damping, self.stiffness_damping))
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

        if self.collider_enabled {
            let (mesh, ids_map, parts_map) = vol.boundary_polyline();
            vol.renumber_dofs(&ids_map);
            let _ = DeformableColliderDesc::new(ShapeHandle::new(mesh))
                .body_parts_mapping(Some(Arc::new(parts_map)))
                .build_with_infos(&vol, cworld);
        }

        vol
    }
}