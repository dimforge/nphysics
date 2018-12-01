use std::ops::AddAssign;
use std::iter;
use std::collections::HashMap;

use alga::linear::FiniteDimInnerSpace;
use na::{self, Real, Point2, Point3, Point4, Vector3, Vector6, Matrix2, Matrix3, Matrix3x2, Matrix2x3, DMatrix,
         DVector, DVectorSlice, DVectorSliceMut, Cholesky, Dynamic, VectorSliceMut2, Vector2,
         Rotation3, Unit};
use ncollide::utils::{self, DeterministicState};
use ncollide::shape::{Polyline, DeformationsType, TrianglePointLocation, Triangle};
use ncollide::query::PointQueryWithLocation;

use object::{Body, BodyPart, BodyHandle, BodyPartHandle, BodyStatus, ActivationStatus};
use solver::{IntegrationParameters, ForceDirection};
use math::{Force, Inertia, Velocity, Matrix, Dim, DIM, Point, Isometry, SpatialVector, RotationMatrix, Vector};

/// One element of a deformable surface.
#[derive(Clone)]
pub struct TriangularElement<N: Real> {
    handle: Option<BodyPartHandle>,
    indices: Point3<usize>,
    com: Point<N>,
    rot: RotationMatrix<N>,
    j: Matrix<N>,
    j_inv: Matrix<N>,
    plastic_strain: SpatialVector<N>,
    surface: N,
    density: N,
}

/// A deformable surface using FEM to simulate linear elasticity.
///
/// The surface is described by a set of triangle elements. This
/// implements an isoparametric approach where the interpolations are linear.
#[derive(Clone)]
pub struct DeformableSurface<N: Real> {
    handle: Option<BodyHandle>,
    elements: Vec<TriangularElement<N>>,
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

    companion_id: usize,
    activation: ActivationStatus<N>,
    status: BodyStatus,
}

impl<N: Real> DeformableSurface<N> {
    /// Initializes a new deformable surface from its triangle elements.
    pub fn new(vertices: &Vec<Point<N>>, triangles: &Vec<Point3<usize>>, density: N, young_modulus: N, poisson_ratio: N, damping_coeffs: (N, N)) -> Self {
        let elements = triangles.iter().map(|idx|
            TriangularElement {
                handle: None,
                indices: idx * DIM,
                com: Point::origin(),
                rot: RotationMatrix::identity(),
                j: na::zero(),
                j_inv: na::zero(),
                plastic_strain: SpatialVector::zeros(),
                surface: na::zero(),
                density,
            }).collect();

        let ndofs = vertices.len() * DIM;
        let rest_positions = DVector::from_iterator(ndofs, vertices.iter().flat_map(|p| p.iter().cloned()));

        DeformableSurface {
            handle: None,
            elements,
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
            activation: ActivationStatus::new_active(),
            status: BodyStatus::Dynamic,
        }
    }

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
        &mut self.velocities
    }


    /// Sets the plastic properties of this deformable surface.
    pub fn set_plasticity(&mut self, strain_threshold: N, creep: N, max_force: N) {
        self.plasticity_threshold = strain_threshold;
        self.plasticity_creep = creep;
        self.plasticity_max_force = max_force;
    }

    fn assemble_mass_with_damping(&mut self, params: &IntegrationParameters<N>) {
        let mass_damping = params.dt * self.damping_coeffs.0;

        for elt in self.elements.iter().filter(|e| e.surface > N::zero()) {
            let coeff_mass = elt.density * elt.surface / na::convert::<_, N>(20.0f64) * (N::one() + mass_damping);

            for a in 0..3 {
                for b in 0..3 {
                    let mass_contribution;

                    if a == b {
                        mass_contribution = coeff_mass * na::convert(2.0);
                    } else {
                        mass_contribution = coeff_mass;
                    }

                    let ia = elt.indices[a];
                    let ib = elt.indices[b];

                    let mut node_mass = self.augmented_mass.fixed_slice_mut::<Dim, Dim>(ia, ib);
                    for i in 0..DIM {
                        node_mass[(i, i)] += mass_contribution;
                    }
                }
            }
        }
    }

    fn assemble_stiffness_and_forces_with_damping(&mut self, gravity: &Vector<N>, params: &IntegrationParameters<N>) {
        let _1: N = na::one();
        let _2: N = na::convert(2.0);
        let dt = params.dt;
        let dt2 = params.dt * params.dt;
        let stiffness_coeff = params.dt * (params.dt + self.damping_coeffs.1);

        // External forces.
        for elt in self.elements.iter().filter(|e| e.surface > N::zero()) {
            let contribution = gravity * (elt.density * elt.surface * na::convert::<_, N>(1.0 / 3.0));

            for k in 0..3 {
                let mut forces_part = self.accelerations.fixed_rows_mut::<Dim>(elt.indices[k]);
                forces_part += contribution;
            }
        }

        /// Internal forces and stiffness.
        let d0 = (self.young_modulus * (_1 - self.poisson_ratio)) / ((_1 + self.poisson_ratio) * (_1 - _2 * self.poisson_ratio));
        let d1 = (self.young_modulus * self.poisson_ratio) / ((_1 + self.poisson_ratio) * (_1 - _2 * self.poisson_ratio));
        let d2 = (self.young_modulus * (_1 - _2 * self.poisson_ratio)) / (_2 * (_1 + self.poisson_ratio) * (_1 - _2 * self.poisson_ratio));

        for elt in self.elements.iter_mut().filter(|e| e.surface > N::zero()) {
            let j_inv_rot = elt.rot.inverse().matrix() * elt.j_inv;

            // XXX: simplify/optimize those two parts.

            /*
             *
             * Plastic strain.
             *
             */
            let mut total_strain = SpatialVector::zeros();

            // Compute plastic strain.
            for a in 0..3 {
                let bn;
                let cn;

                if a == 0 {
                    bn = -j_inv_rot.m11 - j_inv_rot.m12;
                    cn = -j_inv_rot.m21 - j_inv_rot.m22;
                } else {
                    bn = j_inv_rot[(0, a - 1)];
                    cn = j_inv_rot[(1, a - 1)];
                }

                let _0 = N::zero();
                let B_n = Matrix3x2::new(
                    bn, _0,
                    _0, cn,
                    cn, bn,
                );

                let ia = elt.indices[a];
                let pos_part = self.positions.fixed_rows::<Dim>(ia);
                let ref_pos_part = self.rest_positions.fixed_rows::<Dim>(ia);
                let rot_tr = elt.rot.transpose();
                let dpos = rot_tr * pos_part - ref_pos_part;
                total_strain += B_n * dpos;
            }

            let strain = total_strain - elt.plastic_strain;
            if strain.norm() > self.plasticity_threshold {
                let coeff = params.dt * (N::one() / params.dt).min(self.plasticity_creep);
                elt.plastic_strain += strain * coeff;
            }

            if let Some((dir, strain)) = Unit::try_new_and_get(elt.plastic_strain, N::zero()) {
                if strain > self.plasticity_max_force {
                    elt.plastic_strain = *dir * strain;
                }
            }

            // Apply plastic strain.
            for a in 0..3 {
                let bn;
                let cn;

                if a == 0 {
                    bn = -j_inv_rot.m11 - j_inv_rot.m12;
                    cn = -j_inv_rot.m21 - j_inv_rot.m22;
                } else {
                    bn = j_inv_rot[(0, a - 1)];
                    cn = j_inv_rot[(1, a - 1)];
                }

                let _0 = N::zero();

                let P_n = Matrix2x3::new(
                    bn * d0, bn * d1, cn * d2,
                    cn * d1, cn * d0, bn * d2,
                ) * elt.surface;

                let ia = elt.indices[a];
                let mut force_part = self.accelerations.fixed_rows_mut::<Dim>(ia);
                let plastic_force = elt.rot * (P_n * elt.plastic_strain);
                force_part += plastic_force;
            }


            /*
             *
             * Elastic strain.
             *
             */
            for a in 0..3 {
                let bn;
                let cn;

                if a == 0 {
                    bn = -j_inv_rot.m11 - j_inv_rot.m12;
                    cn = -j_inv_rot.m21 - j_inv_rot.m22;
                } else {
                    bn = j_inv_rot[(0, a - 1)];
                    cn = j_inv_rot[(1, a - 1)];
                }

                let ia = elt.indices[a];
                let mut force_part = self.accelerations.fixed_rows_mut::<Dim>(ia);


                for b in 0..3 {
                    let bm;
                    let cm;

                    if b == 0 {
                        bm = -j_inv_rot.m11 - j_inv_rot.m12;
                        cm = -j_inv_rot.m21 - j_inv_rot.m22;
                    } else {
                        bm = j_inv_rot[(0, b - 1)];
                        cm = j_inv_rot[(1, b - 1)];
                    }

                    let node_stiffness = Matrix::new(
                        d0 * bn * bm + d2 * cn * cm, d1 * bn * cm + d2 * cn * bm,
                        d1 * cn * bm + d2 * bn * cm, d0 * cn * cm + d2 * bn * bm,
                    ) * elt.surface;

                    let rot_stiffness = elt.rot * node_stiffness;
                    let rot_tr = elt.rot.transpose();

                    let ib = elt.indices[b];

                    let mut mass_part = self.augmented_mass.fixed_slice_mut::<Dim, Dim>(ia, ib);
                    mass_part.gemm(stiffness_coeff, &rot_stiffness, rot_tr.matrix(), N::one());

                    let vel_part = self.velocities.fixed_rows::<Dim>(ib);
                    let pos_part = self.positions.fixed_rows::<Dim>(ib);
                    let ref_pos_part = self.rest_positions.fixed_rows::<Dim>(ib);
                    let dpos = rot_tr * (vel_part * dt + pos_part) - ref_pos_part;
                    force_part.gemv(-N::one(), &rot_stiffness, &dpos, N::one());
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

        for (idx, part_id) in &mut indices {
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
        let mut new_positions = self.positions.clone();
        let mut new_rest_positions = self.rest_positions.clone();

        for (mesh_i, vol_i) in deformation_indices.iter().cloned().enumerate() {
            let mesh_i = mesh_i * DIM;

            if vol_i >= deformation_indices.len() * DIM {
                dof_map.swap(vol_i, mesh_i);

                for k in 0..DIM {
                    new_positions.swap((mesh_i + k, 0), (vol_i + k, 0));
                    new_rest_positions.swap((mesh_i + k, 0), (vol_i + k, 0));
                }
            } else {
                dof_map[vol_i] = mesh_i;


                for k in 0..DIM {
                    new_positions[(mesh_i + k, 0)] = self.positions[(vol_i + k, 0)];
                    new_rest_positions[(mesh_i + k, 0)] = self.rest_positions[(vol_i + k, 0)];
                }
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
    pub fn quad(pos: &Isometry<N>, extents: &Vector2<N>, nx: usize, ny: usize, density: N, young_modulus: N, poisson_ratio: N, damping_coeffs: (N, N)) -> Self {
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
                coords.x = x * extents.x;
                coords.y = y * extents.y;
                vertices.push(pos * Point::from_coordinates(coords))
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

        Self::new(&vertices, &indices, density, young_modulus, poisson_ratio, damping_coeffs)
    }
}

impl<N: Real> Body<N> for DeformableSurface<N> {
    #[inline]
    fn deformed_positions(&self) -> Option<(DeformationsType, &[N])> {
        Some((DeformationsType::Vectors, self.positions.as_slice()))
    }

    #[inline]
    fn deformed_positions_mut(&mut self) -> Option<(DeformationsType, &mut [N])> {
        Some((DeformationsType::Vectors, self.positions.as_mut_slice()))
    }

    fn update_kinematics(&mut self) {
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
            elt.j_inv = elt.j.try_inverse().unwrap_or(Matrix2::identity());
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
        }
    }

    /// Update the dynamics property of this deformable surface.
    fn update_dynamics(&mut self,
                       gravity: &Vector<N>,
                       params: &IntegrationParameters<N>) {
        self.assemble_mass_with_damping(params);
        self.assemble_stiffness_and_forces_with_damping(gravity, params);

        // FIXME: avoid allocation inside Cholesky at each timestep.
        self.inv_augmented_mass = Cholesky::new(self.augmented_mass.clone()).expect("Singular system found.");
        self.inv_augmented_mass.solve_mut(&mut self.accelerations);
    }

    fn clear_dynamics(&mut self) {
        self.augmented_mass.fill(N::zero());
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
        let ndofs = self.velocities.len();
        DVectorSliceMut::from_slice(self.velocities.as_mut_slice(), ndofs)
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

    fn set_deactivation_threshold(&mut self, threshold: Option<N>) {
        self.activation.set_deactivation_threshold(threshold)
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

    fn inv_mass_mul_generalized_forces(&self, generalized_forces: &mut [N]) {
        let mut out = DVectorSliceMut::from_slice(generalized_forces, self.ndofs());
        self.inv_augmented_mass.solve_mut(&mut out);
    }

    fn body_part_jacobian_mul_unit_force(&self, part: &BodyPart<N>, pt: &Point<N>, force_dir: &ForceDirection<N>, out: &mut [N]) {
        // Needed by the non-linear SOR-prox.
        // FIXME: should this `fill` be done by the non-linear SOR-prox itself?
        DVectorSliceMut::from_slice(out, self.ndofs()).fill(N::zero());

        if let ForceDirection::Linear(dir) = force_dir {
            let elt = part.downcast_ref::<TriangularElement<N>>().expect("The provided body part must be a triangle element");

            let a = self.positions.fixed_rows::<Dim>(elt.indices.x).into_owned();
            let b = self.positions.fixed_rows::<Dim>(elt.indices.y).into_owned();
            let c = self.positions.fixed_rows::<Dim>(elt.indices.z).into_owned();

            let tri = Triangle::new(
                Point::from_coordinates(a),
                Point::from_coordinates(b),
                Point::from_coordinates(c)
            );

            // XXX: This is extremely costly!
            let proj = tri.project_point_with_location(&Isometry::identity(), pt, true).1;

            let bcoords = if let Some(b) = proj.barycentric_coordinates() {
                b
            } else {
                let pt_a = pt.coords - a;
                // XXX: we have to update j_inv to use the new positions here!
                let b = elt.j_inv.tr_mul(&pt_a);
                [(N::one() - b.x - b.y), b.x, b.y]
            };


            VectorSliceMut2::from_slice(&mut out[elt.indices.x..]).copy_from(&(**dir * bcoords[0]));
            VectorSliceMut2::from_slice(&mut out[elt.indices.y..]).copy_from(&(**dir * bcoords[1]));
            VectorSliceMut2::from_slice(&mut out[elt.indices.z..]).copy_from(&(**dir * bcoords[2]));
        }
    }

    fn body_part_point_velocity(&self, part: &BodyPart<N>, point: &Point<N>, force_dir: &ForceDirection<N>) -> N {
        unimplemented!()
        /*
        if let ForceDirection::Linear(dir) = force_dir {
            let elt = part.downcast_ref::<TriangularElement<N>>().expect("The provided body part must be a triangle element");

            let a = self.positions.fixed_rows::<Dim>(elt.indices.x).into_owned();
            let b = self.positions.fixed_rows::<Dim>(elt.indices.y).into_owned();
            let c = self.positions.fixed_rows::<Dim>(elt.indices.z).into_owned();

            let vs = [
                self.velocities.fixed_rows::<Dim>(elt.indices.x).into_owned(),
                self.velocities.fixed_rows::<Dim>(elt.indices.y).into_owned(),
                self.velocities.fixed_rows::<Dim>(elt.indices.z).into_owned()
            ];

            let tri = Triangle::new(
                Point::from_coordinates(a),
                Point::from_coordinates(b),
                Point::from_coordinates(c)
            );

            // XXX: This is extremely costly!
            match tri.project_point_with_location(&Isometry::identity(), point, true).1 {
                TrianglePointLocation::OnVertex(i) => {
                    vs[i].dot(dir)
                }
                TrianglePointLocation::OnEdge(i, uv) => {
                    let idx = Tetrahedron::<N>::edge_ids(i);
                    vs[idx.0].dot(dir) * uv[0] + vs[idx.1].dot(dir) * uv[1]
                }
                TrianglePointLocation::OnFace(i, uvw) => {
                    let idx = Tetrahedron::<N>::face_ids(i);
                    vs[idx.0].dot(dir) * uvw[0] + vs[idx.1].dot(dir) * uvw[1] + vs[idx.2].dot(dir) * uvw[2]
                }
                TrianglePointLocation::OnSolid => {
                    let pt_a = point.coords - a;
                    // XXX: we have to update j_inv to use the new positions here!
                    let bcoords = elt.j_inv.tr_mul(&pt_a);


                    let f2 = bcoords.x;
                    let f3 = bcoords.y;
                    let f4 = bcoords.z;
                    let f1 = N::one() - f2 - f3 - f4;

                    vs[0].dot(dir) * f1 + vs[1].dot(dir) * f2 + vs[2].dot(dir) * f3 + vs[3].dot(dir) * f4
                }
            }
        } else {
            N::zero()
        }
        */
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


impl<N: Real> BodyPart<N> for TriangularElement<N> {
    fn handle(&self) -> Option<BodyPartHandle> {
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

    fn apply_force(&mut self, force: &Force<N>) {
        unimplemented!()
    }
}