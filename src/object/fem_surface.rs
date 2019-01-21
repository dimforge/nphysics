use std::ops::AddAssign;
use std::iter;
use std::collections::HashMap;
use std::sync::Arc;
use std::any::Any;
use either::Either;

use alga::linear::FiniteDimInnerSpace;
use na::{self, Real, Point2, Point3, Vector3, Matrix2, Matrix2x3, DMatrix, U2,
         DVector, DVectorSlice, DVectorSliceMut, Cholesky, Dynamic, Vector2, Unit};
use ncollide::utils::{self, DeterministicState};
use ncollide::shape::{Polyline, DeformationsType, ShapeHandle};

use crate::object::{Body, BodyPart, BodyHandle, BodyPartHandle, BodyStatus, ActivationStatus,
                    FiniteElementIndices, DeformableColliderDesc, BodyDesc, BodyUpdateStatus};
use crate::solver::{IntegrationParameters, ForceDirection};
use crate::math::{Force, Inertia, Velocity, Matrix, Dim, DIM, Point, Isometry, SpatialVector, RotationMatrix, Vector, Translation};
use crate::object::fem_helper;
use crate::world::{World, ColliderWorld};

/// One element of a deformable surface.
#[derive(Clone)]
pub struct TriangularElement<N: Real> {
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
pub struct FEMSurface<N: Real> {
    handle: BodyHandle,
    elements: Vec<TriangularElement<N>>,
    kinematic_nodes: DVector<bool>,
    positions: DVector<N>,
    velocities: DVector<N>,
    accelerations: DVector<N>,
    augmented_mass: DMatrix<N>,
    inv_augmented_mass: Cholesky<N, Dynamic>,

    // Cache.
    // FIXME: use a workspace common with other deformables?
    dpos: DVector<N>,

    // Parameters
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

impl<N: Real> FEMSurface<N> {
    /// Initializes a new deformable surface from its triangle elements.
    fn new(handle: BodyHandle, vertices: &[Point<N>], triangles: &[Point3<usize>], pos: &Isometry<N>,
           scale: &Vector<N>, density: N, young_modulus: N, poisson_ratio: N, damping_coeffs: (N, N)) -> Self {
        let elements = triangles.iter().enumerate().map(|(i, idx)|
            TriangularElement {
                handle: BodyPartHandle(handle, i),
                indices: idx * DIM,
                com: Point::origin(),
                rot: RotationMatrix::identity(),
                inv_rot: RotationMatrix::identity(),
                j: na::zero(),
                local_j_inv: na::zero(),
                total_strain: SpatialVector::zeros(),
                plastic_strain: SpatialVector::zeros(),
                surface: na::zero(),
                density,
            }).collect();

        let ndofs = vertices.len() * DIM;
        let mut rest_positions = DVector::zeros(ndofs);

        for (i, pt)  in vertices.iter().enumerate() {
            let pt = pos * Point::from_coordinates(pt.coords.component_mul(&scale));
            rest_positions.fixed_rows_mut::<Dim>(i * DIM).copy_from(&pt.coords);
        }

        let (d0, d1, d2) = fem_helper::elasticity_coefficients(young_modulus, poisson_ratio);

        FEMSurface {
            handle,
            elements,
            kinematic_nodes: DVector::repeat(vertices.len(), false),
            positions: rest_positions.clone(),
            velocities: DVector::zeros(ndofs),
            accelerations: DVector::zeros(ndofs),
            augmented_mass: DMatrix::zeros(ndofs, ndofs),
            inv_augmented_mass: Cholesky::new(DMatrix::zeros(0, 0)).unwrap(),
            dpos: DVector::zeros(ndofs),
            rest_positions,
            damping_coeffs,
            young_modulus,
            poisson_ratio,
            companion_id: 0,
            plasticity_threshold: N::zero(),
            plasticity_max_force: N::zero(),
            plasticity_creep: N::zero(),
            d0, d1, d2,
            activation: ActivationStatus::new_active(),
            status: BodyStatus::Dynamic,
            update_status: BodyUpdateStatus::new(),
            user_data: None
        }
    }

    user_data_accessors!();

    /// The position of this body in generalized coordinates.
    #[inline]
    pub fn positions(&self) -> &DVector<N> {
        &self.positions
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

    pub fn handle(&self) -> BodyHandle {
        self.handle
    }

    fn assemble_mass_with_damping(&mut self, params: &IntegrationParameters<N>) {
        let mass_damping = params.dt * self.damping_coeffs.0;

        for elt in self.elements.iter().filter(|e| e.surface > N::zero()) {
            let coeff_mass = elt.density * elt.surface / na::convert::<_, N>(20.0f64) * (N::one() + mass_damping);

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


    fn assemble_stiffness(&mut self, params: &IntegrationParameters<N>) {
        let _1: N = na::one();
        let _2: N = na::convert(2.0);
        let dt = params.dt;
        let stiffness_coeff = params.dt * (params.dt + self.damping_coeffs.1);

        for elt in self.elements.iter_mut().filter(|e| e.surface > N::zero()) {
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
                    // let P_n = Matrix3x6::new(
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
        let stiffness_coeff = params.dt * (params.dt + self.damping_coeffs.1);

        // External forces.
        for elt in self.elements.iter().filter(|e| e.surface > N::zero()) {
            let contribution = gravity * (elt.density * elt.surface * na::convert::<_, N>(1.0 / 3.0));

            for k in 0..3 {
                let ie = elt.indices[k];

                if !self.kinematic_nodes[ie / DIM] {
                    let mut forces_part = self.accelerations.fixed_rows_mut::<Dim>(ie);
                    forces_part += contribution;
                }
            }
        }

        for elt in self.elements.iter_mut().filter(|e| e.surface > N::zero()) {

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

                    let mut force_part = self.accelerations.fixed_rows_mut::<Dim>(ia);

                    // Fields of P_n * elt.volume:
                    //
                    // let P_n = Matrix3x6::new(
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
            new_positions.fixed_rows_mut::<U2>(target_i).copy_from(&self.positions.fixed_rows::<U2>(orig_i));
            new_rest_positions.fixed_rows_mut::<U2>(target_i).copy_from(&self.rest_positions.fixed_rows::<U2>(orig_i));
            dof_map[orig_i] = target_i;
            remapped[orig_i] = true;
        }

        let mut curr_target = deformation_indices.len() * 2;

        for orig_i in (0..self.positions.len()).step_by(2) {
            if !remapped[orig_i] {
                new_positions.fixed_rows_mut::<U2>(curr_target).copy_from(&self.positions.fixed_rows::<U2>(orig_i));
                new_rest_positions.fixed_rows_mut::<U2>(curr_target).copy_from(&self.rest_positions.fixed_rows::<U2>(orig_i));
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
                vertices.push(Point::from_coordinates(coords))
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
        self.update_status.set_local_inertia_changed(true);
        self.kinematic_nodes[i] = is_kinematic;
    }

    /// Mark all nodes as non-kinematic.
    pub fn clear_kinematic_nodes(&mut self) {
        self.update_status.set_local_inertia_changed(true);
        self.kinematic_nodes.fill(false)
    }
}

impl<N: Real> Body<N> for FEMSurface<N> {
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
            let j_det =  elt.j.determinant();
            elt.surface = j_det * na::convert(1.0 / 2.0);
            elt.com = Point::from_coordinates(a + b + c) * na::convert::<_, N>(1.0 / 3.0);

            /*
             * Compute the orientation.
             */
            // Undeformed points.

            // FIXME: stiffness wrapping
            let rest_a = self.rest_positions.fixed_rows::<Dim>(elt.indices.x);
            let rest_b = self.rest_positions.fixed_rows::<Dim>(elt.indices.y);
            let rest_c = self.rest_positions.fixed_rows::<Dim>(elt.indices.z);

            let rest_ab = rest_b - rest_a;
            let rest_ac = rest_c - rest_a;

            let inv_e = Matrix::from_columns(&[rest_ab, rest_ac]).transpose().try_inverse().unwrap_or(Matrix2::identity());

            // The transformation matrix from which we can get the rotation component.
            let g = (inv_e * elt.j).transpose();
            // FIXME: optimize this.
            let mut cols = [
                g.column(0).into_owned(),
                g.column(1).into_owned(),
            ];
            let _ = Vector::orthonormalize(&mut cols);
            elt.rot = RotationMatrix::from_matrix_unchecked(Matrix::from_columns(&cols));
            elt.inv_rot = elt.rot.inverse();

            let j_inv = elt.j.try_inverse().unwrap_or(Matrix2::identity());
            let local_j_inv = elt.inv_rot.matrix() * j_inv;
            elt.local_j_inv = Matrix2x3::new(
                -local_j_inv.m11 - local_j_inv.m12, local_j_inv.m11, local_j_inv.m12,
                -local_j_inv.m21 - local_j_inv.m22, local_j_inv.m21, local_j_inv.m22,
            );

        }
    }

    /// Update the dynamics property of this deformable surface.
    fn update_dynamics(&mut self,
                       gravity: &Vector<N>,
                       params: &IntegrationParameters<N>) {
        if self.update_status.inertia_needs_update() {
            self.augmented_mass.fill(N::zero());
            self.assemble_mass_with_damping(params);
            self.assemble_stiffness(params);

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
        self.accelerations.fill(N::zero());
        self.assemble_forces(gravity, params);
        self.inv_augmented_mass.solve_mut(&mut self.accelerations);
    }

    fn clear_dynamics(&mut self) {
    }

    fn clear_forces(&mut self) {
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
        self.update_status.set_velocity_changed(true);
        self.activation.set_energy(N::zero());
        self.velocities.fill(N::zero());
    }

    fn set_deactivation_threshold(&mut self, threshold: Option<N>) {
        self.activation.set_deactivation_threshold(threshold)
    }

    fn part(&self, id: usize) -> Option<&BodyPart<N>> {
        self.elements.get(id).map(|b| b as &BodyPart<N>)
    }

    fn part_mut(&mut self, id: usize) -> Option<&mut BodyPart<N>> {
        self.elements.get_mut(id).map(|b| b as &mut BodyPart<N>)
    }

    fn world_point_at_material_point(&self, part: &BodyPart<N>, point: &Point<N>) -> Point<N> {
        let elt = part.downcast_ref::<TriangularElement<N>>().expect("The provided body part must be a triangular element");
        fem_helper::world_point_at_material_point(FiniteElementIndices::Triangle(elt.indices), &self.positions, point)
    }

    fn position_at_material_point(&self, part: &BodyPart<N>, point: &Point<N>) -> Isometry<N> {
        let elt = part.downcast_ref::<TriangularElement<N>>().expect("The provided body part must be a triangular element");
        let pt = fem_helper::world_point_at_material_point(FiniteElementIndices::Triangle(elt.indices), &self.positions, point);
        Isometry::from_parts(Translation::from_vector(pt.coords), na::one())
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
    fn setup_internal_velocity_constraints(&mut self, _: &mut DVectorSliceMut<N>) {}

    #[inline]
    fn step_solve_internal_velocity_constraints(&mut self, _: &mut DVectorSliceMut<N>) {}

    #[inline]
    fn step_solve_internal_position_constraints(&mut self, _: &IntegrationParameters<N>) {}
}


impl<N: Real> BodyPart<N> for TriangularElement<N> {
    fn part_handle(&self) -> BodyPartHandle {
        self.handle
    }

    fn center_of_mass(&self) -> Point<N> {
        self.com
    }

    fn position(&self) -> Isometry<N> {
        Isometry::new(self.com.coords, na::zero())
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

    fn apply_force(&mut self, _: &Force<N>) {
        unimplemented!()
    }
}


enum FEMSurfaceDescGeometry<'a, N: Real> {
    Quad(usize, usize),
    Triangles(&'a [Point<N>], &'a [Point3<usize>])
}

pub struct FEMSurfaceDesc<'a, N: Real> {
    geom: FEMSurfaceDescGeometry<'a, N>,
    scale: Vector<N>,
    position: Isometry<N>,
    young_modulus: N,
    poisson_ratio: N,
    sleep_threshold: Option<N>,
    boundary_polyline_collider_enabled: bool,
    mass_damping: N,
    stiffness_damping: N,
    density: N,
    plasticity: (N, N, N),
    kinematic_nodes: Vec<usize>,
    status: BodyStatus
}

impl<'a, N: Real> FEMSurfaceDesc<'a, N> {
    fn with_geometry(geom: FEMSurfaceDescGeometry<'a, N>) -> Self {
        FEMSurfaceDesc {
            geom,
            scale: Vector::repeat(N::one()),
            position: Isometry::identity(),
            young_modulus: na::convert(1.0e3),
            poisson_ratio: N::zero(),
            sleep_threshold: Some(ActivationStatus::default_threshold()),
            boundary_polyline_collider_enabled: false,
            mass_damping: na::convert(0.2),
            stiffness_damping: N::zero(),
            density: N::one(),
            plasticity: (N::zero(), N::zero(), N::zero()),
            kinematic_nodes: Vec::new(),
            status: BodyStatus::Dynamic
        }
    }

    pub fn new(vertices: &'a [Point<N>], triangles: &'a [Point3<usize>]) -> Self {
        Self::with_geometry(FEMSurfaceDescGeometry::Triangles(vertices, triangles))
    }

    pub fn quad(subdiv_x: usize, subdiv_y: usize) -> Self {
        Self::with_geometry(FEMSurfaceDescGeometry::Quad(subdiv_x, subdiv_y))
    }

    pub fn clear_kinematic_nodes(&mut self) -> &mut Self {
        self.kinematic_nodes.clear();
        self
    }

    desc_custom_setters!(
        self.with_boundary_polyline_collider, set_boundary_polyline_collider_enabled, enable: bool | { self.boundary_polyline_collider_enabled = enable }
        self.with_plasticity, set_plasticity, strain_threshold: N, creep: N, max_force: N | { self.plasticity = (strain_threshold, creep, max_force) }
        self.with_kinematic_nodes, set_kinematic_nodes, nodes: &[usize] | { self.kinematic_nodes.extend_from_slice(nodes) }
        self.with_translation, set_translation, vector: Vector<N> | { self.position.translation.vector = vector }
    );

    desc_setters!(
        with_scale, set_scale, scale: Vector<N>
        with_young_modulus, set_young_modulus, young_modulus: N
        with_poisson_ratio, set_poisson_ratio, poisson_ratio: N
        with_sleep_threshold, set_sleep_threshold, sleep_threshold: Option<N>
        with_mass_damping, set_mass_damping, mass_damping: N
        with_stiffness_damping, set_stiffness_damping, stiffness_damping: N
        with_density, set_density, density: N
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
        [val] young_modulus: N
        [val] poisson_ratio: N
        [val] sleep_threshold: Option<N>
        [val] mass_damping: N
        [val] stiffness_damping: N
        [val] density: N
        [val] status: BodyStatus
        [val] boundary_polyline_collider_enabled: bool
        [ref] position: Isometry<N>
        [ref] scale: Vector<N>
    );

    pub fn build<'w>(&self, world: &'w mut World<N>) -> &'w FEMSurface<N> {
        world.add_body(self)
    }
}

impl<'a, N: Real> BodyDesc<N> for FEMSurfaceDesc<'a, N> {
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

        for i in &self.kinematic_nodes {
            vol.set_node_kinematic(*i, true)
        }

        if self.boundary_polyline_collider_enabled {
            let (mesh, ids_map, parts_map) = vol.boundary_polyline();
            vol.renumber_dofs(&ids_map);
            let _ = DeformableColliderDesc::new(ShapeHandle::new(mesh))
                .with_body_parts_mapping(Some(Arc::new(parts_map)))
                .build_with_infos(&vol, cworld);
        }

        vol
    }
}