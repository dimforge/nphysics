use std::ops::AddAssign;
use std::iter;
use std::collections::HashMap;
use either::Either;

use alga::linear::FiniteDimInnerSpace;
use na::{self, Real, Point3, Point4, Vector3, Vector6, Matrix3, Matrix6x3, Matrix3x6, DMatrix, Isometry3,
         DVector, DVectorSlice, DVectorSliceMut, Cholesky, Dynamic, U3, VectorSliceMut2,
         Rotation3, Unit, VectorSliceMut3};
use ncollide::utils::{self, DeterministicState};
use ncollide::shape::{TriMesh, DeformationsType, TetrahedronPointLocation, Tetrahedron};
use ncollide::query::PointQueryWithLocation;

use object::{Body, BodyPart, BodyHandle, BodyPartHandle, BodyStatus, ActivationStatus, FiniteElementIndices};
use solver::{IntegrationParameters, ForceDirection};
use math::{Force, Inertia, Velocity};
use object::fem_helper;

/// One element of a deformable volume.
#[derive(Clone)]
pub struct TetrahedralElement<N: Real> {
    handle: Option<BodyPartHandle>,
    indices: Point4<usize>,
    com: Point3<N>,
    rot: Rotation3<N>,
    j: Matrix3<N>,
    j_inv: Matrix3<N>,
    plastic_strain: Vector6<N>,
    volume: N,
    density: N,
}

/// A deformable volume using FEM to simulate linear elasticity.
///
/// The volume is described by a set of tetrahedral elements. This
/// implements an isoparametric approach where the interpolations are linear.
#[derive(Clone)]
pub struct DeformableVolume<N: Real> {
    handle: Option<BodyHandle>,
    elements: Vec<TetrahedralElement<N>>,
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

    companion_id: usize,
    activation: ActivationStatus<N>,
    status: BodyStatus,
}

impl<N: Real> DeformableVolume<N> {
    /// Initializes a new deformable volume from its tetrahedral elements.
    pub fn new(vertices: &Vec<Point3<N>>, tetrahedra: &Vec<Point4<usize>>, density: N, young_modulus: N, poisson_ratio: N, damping_coeffs: (N, N)) -> Self {
        let elements = tetrahedra.iter().map(|idx|
            TetrahedralElement {
                handle: None,
                indices: idx * 3,
                com: Point3::origin(),
                rot: Rotation3::identity(),
                j: na::zero(),
                j_inv: na::zero(),
                plastic_strain: Vector6::zeros(),
                volume: na::zero(),
                density,
            }).collect();

        let ndofs = vertices.len() * 3;
        let rest_positions = DVector::from_iterator(ndofs, vertices.iter().flat_map(|p| p.iter().cloned()));

        DeformableVolume {
            handle: None,
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


    /// Sets the plastic properties of this deformable volume.
    pub fn set_plasticity(&mut self, strain_threshold: N, creep: N, max_force: N) {
        self.plasticity_threshold = strain_threshold;
        self.plasticity_creep = creep;
        self.plasticity_max_force = max_force;
    }

    fn assemble_mass_with_damping(&mut self, params: &IntegrationParameters<N>) {
        let mass_damping = params.dt * self.damping_coeffs.0;

        for elt in self.elements.iter().filter(|e| e.volume > N::zero()) {
            let coeff_mass = elt.density * elt.volume / na::convert::<_, N>(20.0f64) * (N::one() + mass_damping);

            for a in 0..4 {
                for b in 0..4 {
                    let mass_contribution;

                    if a == b {
                        mass_contribution = coeff_mass * na::convert(2.0);
                    } else {
                        mass_contribution = coeff_mass;
                    }

                    let ia = elt.indices[a];
                    let ib = elt.indices[b];

                    let mut node_mass = self.augmented_mass.fixed_slice_mut::<U3, U3>(ia, ib);
                    node_mass[(0, 0)] += mass_contribution;
                    node_mass[(1, 1)] += mass_contribution;
                    node_mass[(2, 2)] += mass_contribution;
                }
            }
        }
    }

    fn assemble_stiffness_and_forces_with_damping(&mut self, gravity: &Vector3<N>, params: &IntegrationParameters<N>) {
        let _1: N = na::one();
        let _2: N = na::convert(2.0);
        let _6: N = na::convert(6.0);
        let dt = params.dt;
        let dt2 = params.dt * params.dt;
        let stiffness_coeff = params.dt * (params.dt + self.damping_coeffs.1);

        // External forces.
        for elt in self.elements.iter().filter(|e| e.volume > N::zero()) {
            let contribution = gravity * (elt.density * elt.volume * na::convert::<_, N>(1.0 / 4.0));

            for k in 0..4 {
                let mut forces_part = self.accelerations.fixed_rows_mut::<U3>(elt.indices[k]);
                forces_part += contribution;
            }
        }

        /// Internal forces and stiffness.
        let d0 = (self.young_modulus * (_1 - self.poisson_ratio)) / ((_1 + self.poisson_ratio) * (_1 - _2 * self.poisson_ratio));
        let d1 = (self.young_modulus * self.poisson_ratio) / ((_1 + self.poisson_ratio) * (_1 - _2 * self.poisson_ratio));
        let d2 = (self.young_modulus * (_1 - _2 * self.poisson_ratio)) / (_2 * (_1 + self.poisson_ratio) * (_1 - _2 * self.poisson_ratio));

        for elt in self.elements.iter_mut().filter(|e| e.volume > N::zero()) {
            let j_inv_rot = elt.rot.inverse().matrix() * elt.j_inv;

            // XXX: simplify/optimize those two parts.

            /*
             *
             * Plastic strain.
             *
             */
            let mut total_strain = Vector6::zeros();

            // Compute plastic strain.
            for a in 0..4 {
                let bn;
                let cn;
                let dn;

                if a == 0 {
                    bn = -j_inv_rot.m11 - j_inv_rot.m12 - j_inv_rot.m13;
                    cn = -j_inv_rot.m21 - j_inv_rot.m22 - j_inv_rot.m23;
                    dn = -j_inv_rot.m31 - j_inv_rot.m32 - j_inv_rot.m33;
                } else {
                    bn = j_inv_rot[(0, a - 1)];
                    cn = j_inv_rot[(1, a - 1)];
                    dn = j_inv_rot[(2, a - 1)];
                }

                let _0 = N::zero();
                let B_n = Matrix6x3::new(
                    bn, _0, _0,
                    _0, cn, _0,
                    _0, _0, dn,
                    cn, bn, _0,
                    dn, _0, bn,
                    _0, dn, cn
                );

                let ia = elt.indices[a];
                let pos_part = self.positions.fixed_rows::<U3>(ia);
                let ref_pos_part = self.rest_positions.fixed_rows::<U3>(ia);
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
            for a in 0..4 {
                let bn;
                let cn;
                let dn;

                if a == 0 {
                    bn = -j_inv_rot.m11 - j_inv_rot.m12 - j_inv_rot.m13;
                    cn = -j_inv_rot.m21 - j_inv_rot.m22 - j_inv_rot.m23;
                    dn = -j_inv_rot.m31 - j_inv_rot.m32 - j_inv_rot.m33;
                } else {
                    bn = j_inv_rot[(0, a - 1)];
                    cn = j_inv_rot[(1, a - 1)];
                    dn = j_inv_rot[(2, a - 1)];
                }

                let _0 = N::zero();

                let P_n = Matrix3x6::new(
                    bn * d0, bn * d1, bn * d1, cn * d2, dn * d2, _0,
                    cn * d1, cn * d0, cn * d1, bn * d2, _0, dn * d2,
                    dn * d1, dn * d1, dn * d0, _0, bn * d2, cn * d2
                ) * elt.volume;

                let ia = elt.indices[a];
                let mut force_part = self.accelerations.fixed_rows_mut::<U3>(ia);
                let plastic_force = elt.rot * (P_n * elt.plastic_strain);
                force_part += plastic_force;
            }


            /*
             *
             * Elastic strain.
             *
             */
            for a in 0..4 {
                let bn;
                let cn;
                let dn;

                if a == 0 {
                    bn = -j_inv_rot.m11 - j_inv_rot.m12 - j_inv_rot.m13;
                    cn = -j_inv_rot.m21 - j_inv_rot.m22 - j_inv_rot.m23;
                    dn = -j_inv_rot.m31 - j_inv_rot.m32 - j_inv_rot.m33;
                } else {
                    bn = j_inv_rot[(0, a - 1)];
                    cn = j_inv_rot[(1, a - 1)];
                    dn = j_inv_rot[(2, a - 1)];
                }

                let ia = elt.indices[a];
                let mut force_part = self.accelerations.fixed_rows_mut::<U3>(ia);


                for b in 0..4 {
                    let bm;
                    let cm;
                    let dm;

                    if b == 0 {
                        bm = -j_inv_rot.m11 - j_inv_rot.m12 - j_inv_rot.m13;
                        cm = -j_inv_rot.m21 - j_inv_rot.m22 - j_inv_rot.m23;
                        dm = -j_inv_rot.m31 - j_inv_rot.m32 - j_inv_rot.m33;
                    } else {
                        bm = j_inv_rot[(0, b - 1)];
                        cm = j_inv_rot[(1, b - 1)];
                        dm = j_inv_rot[(2, b - 1)];
                    }

                    let node_stiffness = Matrix3::new(
                        d0 * bn * bm + d2 * (cn * cm + dn * dm), d1 * bn * cm + d2 * cn * bm, d1 * bn * dm + d2 * dn * bm,
                        d1 * cn * bm + d2 * bn * cm, d0 * cn * cm + d2 * (bn * bm + dn * dm), d1 * cn * dm + d2 * dn * cm,
                        d1 * dn * bm + d2 * bn * dm, d1 * dn * cm + d2 * cn * dm, d0 * dn * dm + d2 * (bn * bm + cn * cm),
                    ) * elt.volume;

                    let rot_stiffness = elt.rot * node_stiffness;
                    let rot_tr = elt.rot.transpose();

                    let ib = elt.indices[b];

                    let mut mass_part = self.augmented_mass.fixed_slice_mut::<U3, U3>(ia, ib);
                    mass_part.gemm(stiffness_coeff, &rot_stiffness, rot_tr.matrix(), N::one());

                    let vel_part = self.velocities.fixed_rows::<U3>(ib);
                    let pos_part = self.positions.fixed_rows::<U3>(ib);
                    let ref_pos_part = self.rest_positions.fixed_rows::<U3>(ib);
                    let dpos = rot_tr * (vel_part * dt + pos_part) - ref_pos_part;
                    force_part.gemv(-N::one(), &rot_stiffness, &dpos, N::one());
                }
            }
        }
    }

    /// Returns the triangles at the boundary of this volume.
    ///
    /// Each element of the returned `Vec` is a tuple containing the 3 indices of the triangle
    /// vertices, and the index of the corresponding tetrahedral element.
    pub fn boundary(&self) -> Vec<(Point3<usize>, usize)> {
        fn key(a: usize, b: usize, c: usize) -> (usize, usize, usize) {
            let (sa, sb, sc) = utils::sort3(&a, &b, &c);
            (*sa, *sb, *sc)
        }

        let mut faces = HashMap::with_hasher(DeterministicState::new());

        for (i, elt) in self.elements.iter().enumerate() {
            let k1 = key(elt.indices.x, elt.indices.y, elt.indices.z);
            let k2 = key(elt.indices.y, elt.indices.z, elt.indices.w);
            let k3 = key(elt.indices.z, elt.indices.w, elt.indices.x);
            let k4 = key(elt.indices.w, elt.indices.x, elt.indices.y);

            faces.entry(k1).or_insert((0, elt.indices.w, i)).0.add_assign(1);
            faces.entry(k2).or_insert((0, elt.indices.x, i)).0.add_assign(1);
            faces.entry(k3).or_insert((0, elt.indices.y, i)).0.add_assign(1);
            faces.entry(k4).or_insert((0, elt.indices.z, i)).0.add_assign(1);
        }

        let boundary = faces.iter().filter_map(|(k, n)| {
            if n.0 == 1 {
                // Ensure the triangle has an outward normal.
                // FIXME: there is a much more efficient way of doing this, given the
                // tetrahedra orientations and the face.
                let a = self.positions.fixed_rows::<U3>(k.0);
                let b = self.positions.fixed_rows::<U3>(k.1);
                let c = self.positions.fixed_rows::<U3>(k.2);
                let d = self.positions.fixed_rows::<U3>(n.1);

                let ab = b - a;
                let ac = c - a;
                let ad = d - a;

                if ab.cross(&ac).dot(&ad) < N::zero() {
                    Some((Point3::new(k.0, k.1, k.2), n.2))
                } else {
                    Some((Point3::new(k.0, k.2, k.1), n.2))
                }
            } else {
                None
            }
        }).collect();

        boundary
    }

    /// Returns a triangle mesh at the boundary of this volume as well as a mapping between the mesh
    /// vertices and this volume degrees of freedom and the mapping between the mesh triangles and
    /// this volume body parts (the tetrahedral elements).
    ///
    /// The output is (triangle mesh, deformation indices, element to body part map).
    pub fn boundary_mesh(&self) -> (TriMesh<N>, Vec<usize>, Vec<usize>) {
        const INVALID: usize = usize::max_value();
        let mut deformation_indices = Vec::new();
        let mut indices = self.boundary();
        let mut idx_remap: Vec<usize> = iter::repeat(INVALID).take(self.positions.len() / 3).collect();
        let mut vertices = Vec::new();

        for (idx, part_id) in &mut indices {
            for i in 0..3 {
                let idx_i = &mut idx[i];
                if idx_remap[*idx_i / 3] == INVALID {
                    let new_id = vertices.len();
                    vertices.push(Point3::new(
                        self.positions[*idx_i + 0],
                        self.positions[*idx_i + 1],
                        self.positions[*idx_i + 2])
                    );
                    deformation_indices.push(*idx_i);
                    idx_remap[*idx_i / 3] = new_id;
                    *idx_i = new_id;
                } else {
                    *idx_i = idx_remap[*idx_i / 3];
                }
            }
        }

        let body_parts = indices.iter().map(|i| i.1).collect();
//        println!("Number of trimesh triangles: {}", indices.len());
        let indices = indices.into_iter().map(|i| i.0).collect();

        (TriMesh::new(vertices, indices, None), deformation_indices, body_parts)
    }

    /// Renumber degrees of freedom so that the `deformation_indices[i]`-th DOF becomes the `i`-th one.
    pub fn renumber_dofs(&mut self, deformation_indices: &[usize]) {
        let mut dof_map: Vec<_> = (0..).take(self.positions.len()).collect();
        let mut new_positions = self.positions.clone();
        let mut new_rest_positions = self.rest_positions.clone();

        for (mesh_i, vol_i) in deformation_indices.iter().cloned().enumerate() {
            let mesh_i = mesh_i * 3;

            if vol_i >= deformation_indices.len() * 3 {
                dof_map.swap(vol_i, mesh_i);

                new_positions.swap((mesh_i + 0, 0), (vol_i + 0, 0));
                new_positions.swap((mesh_i + 1, 0), (vol_i + 1, 0));
                new_positions.swap((mesh_i + 2, 0), (vol_i + 2, 0));

                new_rest_positions.swap((mesh_i + 0, 0), (vol_i + 0, 0));
                new_rest_positions.swap((mesh_i + 1, 0), (vol_i + 1, 0));
                new_rest_positions.swap((mesh_i + 2, 0), (vol_i + 2, 0));
            } else {
                dof_map[vol_i] = mesh_i;
                new_positions[(mesh_i + 0, 0)] = self.positions[(vol_i + 0, 0)];
                new_positions[(mesh_i + 1, 0)] = self.positions[(vol_i + 1, 0)];
                new_positions[(mesh_i + 2, 0)] = self.positions[(vol_i + 2, 0)];

                new_rest_positions[(mesh_i + 0, 0)] = self.rest_positions[(vol_i + 0, 0)];
                new_rest_positions[(mesh_i + 1, 0)] = self.rest_positions[(vol_i + 1, 0)];
                new_rest_positions[(mesh_i + 2, 0)] = self.rest_positions[(vol_i + 2, 0)];
            }
        }

        for elt in &mut self.elements {
            elt.indices.coords.apply(|i| dof_map[i]);
        }

        self.positions = new_positions;
        self.rest_positions = new_rest_positions;
    }

// FIXME: add a method to apply a transformation to the whole volume.

    /// Constructs an axis-aligned cube with regular subdivisions along each axis.
    ///
    /// The cube is subdivided `nx` (resp. `ny` and `nz`) times along
    /// the `x` (resp. `y` and `z`) axis.
    pub fn cube(pos: &Isometry3<N>, extents: &Vector3<N>, nx: usize, ny: usize, nz: usize, density: N, young_modulus: N, poisson_ratio: N, damping_coeffs: (N, N)) -> Self {
        let mut vertices = Vec::new();
        let mut indices = Vec::new();

        // First, generate the vertices.
        let x_step: N = na::convert(1.0 / nx as f64);
        let y_step: N = na::convert(1.0 / ny as f64);
        let z_step: N = na::convert(1.0 / nz as f64);

        for i in 0..=nx {
            let x = x_step * na::convert(i as f64) - na::convert(0.5);

            for j in 0..=ny {
                let y = y_step * na::convert(j as f64) - na::convert(0.5);

                for k in 0..=nz {
                    let z = z_step * na::convert(k as f64) - na::convert(0.5);
                    vertices.push(pos * Point3::new(x * extents.x, y * extents.y, z * extents.z))
                }
            }
        }

        // Second, generate indices.
        for i in 0..nx {
            for j in 0..ny {
                for k in 0..nz {
                    // See https://www.ics.uci.edu/~eppstein/projects/tetra/
                    // for the 5-elements tetrahedral decomposition where local
                    // cube indices are as follows:
                    //
                    //     4 o----------o 7
                    //       | 5 o----------o 6
                    //       |   |      ·   |                y
                    //       |   |      ·   |                ^
                    //     0 o---|· · · · 3 |                |
                    //           o----------o                 --> x
                    //           1          2
                    /* Local cubic indices:
                    let a = Point4::new(0, 1, 2, 5);
                    let b = Point4::new(2, 5, 6, 7);
                    let c = Point4::new(2, 7, 3, 0);
                    let d = Point4::new(7, 4, 0, 5);
                    let e = Point4::new(0, 2, 7, 5);
                    */
                    fn shift(ny: usize, nz: usize, di: usize, dj: usize, dk: usize) -> usize {
                        ((di * (ny + 1) + dj) * (nz + 1)) + dk
                    }
                    // _0 = node at (i, j, k)
                    let _0 = (i * (ny + 1) + j) * (nz + 1) + k;
                    let _1 = _0 + shift(ny, nz, 0, 0, 1);
                    let _2 = _0 + shift(ny, nz, 1, 0, 1);
                    let _3 = _0 + shift(ny, nz, 1, 0, 0);
                    let _4 = _0 + shift(ny, nz, 0, 1, 0);
                    let _5 = _0 + shift(ny, nz, 0, 1, 1);
                    let _6 = _0 + shift(ny, nz, 1, 1, 1);
                    let _7 = _0 + shift(ny, nz, 1, 1, 0);

                    let ifirst = indices.len();

                    if (i % 2) == 0 && ((j % 2) == (k % 2)) ||
                        (i % 2) == 1 && ((j % 2) != (k % 2)) {
                        indices.push(Point4::new(_0, _1, _2, _5));
                        indices.push(Point4::new(_2, _5, _6, _7));
                        indices.push(Point4::new(_2, _7, _3, _0));
                        indices.push(Point4::new(_7, _4, _0, _5));
                        indices.push(Point4::new(_0, _2, _7, _5));
                    } else {
                        indices.push(Point4::new(_4, _6, _5, _1));
                        indices.push(Point4::new(_6, _2, _1, _3));
                        indices.push(Point4::new(_6, _7, _3, _4));
                        indices.push(Point4::new(_3, _4, _0, _1));
                        indices.push(Point4::new(_4, _3, _6, _1));
                    }
                }
            }
        }

        Self::new(&vertices, &indices, density, young_modulus, poisson_ratio, damping_coeffs)
    }
}

impl<N: Real> Body<N> for DeformableVolume<N> {
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
            let a = self.positions.fixed_rows::<U3>(elt.indices.x);
            let b = self.positions.fixed_rows::<U3>(elt.indices.y);
            let c = self.positions.fixed_rows::<U3>(elt.indices.z);
            let d = self.positions.fixed_rows::<U3>(elt.indices.w);

            let ab = b - a;
            let ac = c - a;
            let ad = d - a;

            elt.j = Matrix3::new(
                ab.x, ab.y, ab.z,
                ac.x, ac.y, ac.z,
                ad.x, ad.y, ad.z,
            );
            let j_det =  elt.j.determinant();
            elt.volume = j_det * na::convert(1.0 / 6.0);
            elt.j_inv = elt.j.try_inverse().unwrap_or(Matrix3::identity());
            elt.com = Point3::from_coordinates(a + b + c + d) * na::convert::<_, N>(1.0 / 4.0);

            /*
             * Compute the orientation.
             */
            // Undeformed points.
            let rest_a = self.rest_positions.fixed_rows::<U3>(elt.indices.x);
            let rest_b = self.rest_positions.fixed_rows::<U3>(elt.indices.y);
            let rest_c = self.rest_positions.fixed_rows::<U3>(elt.indices.z);
            let rest_d = self.rest_positions.fixed_rows::<U3>(elt.indices.w);

            let rest_ab = rest_b - rest_a;
            let rest_ac = rest_c - rest_a;
            let rest_ad = rest_d - rest_a;

            let coeff = N::one() / j_det; // 1 / (6V)
            let n1 = rest_ac.cross(&rest_ad) * coeff;
            let n2 = rest_ad.cross(&rest_ab) * coeff;
            let n3 = rest_ab.cross(&rest_ac) * coeff;

            // The transformation matrix from which we can get the rotation component.
            let g = (Matrix3::from_columns(&[n1, n2, n3]) * elt.j).transpose();
            // FIXME: optimize this.
            let mut cols = [
                g.column(0).into_owned(),
                g.column(1).into_owned(),
                g.column(2).into_owned(),
            ];
            let _ = Vector3::orthonormalize(&mut cols);
            elt.rot = Rotation3::from_matrix_unchecked(Matrix3::from_columns(&cols));
        }
    }

    /// Update the dynamics property of this deformable volume.
    fn update_dynamics(&mut self,
                       gravity: &Vector3<N>,
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

    fn fill_constraint_geometry(
        &self,
        part: &BodyPart<N>,
        ndofs: usize, // FIXME: keep this parameter?
        center: &Point3<N>,
        force_dir: &ForceDirection<N>,
        j_id: usize,
        wj_id: usize,
        jacobians: &mut [N],
        inv_r: &mut N,
        ext_vels: Option<&DVectorSlice<N>>,
        out_vel: Option<&mut N>
    ) {
        let elt = part.downcast_ref::<TetrahedralElement<N>>().expect("The provided body part must be a triangular mass-spring element");
        fem_helper::fill_contact_geometry_fem(
            self.ndofs(),
            self.status,
            FiniteElementIndices::Tetrahedron(elt.indices),
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
    fn setup_internal_velocity_constraints(&mut self, dvels: &mut DVectorSliceMut<N>) {}

    #[inline]
    fn step_solve_internal_velocity_constraints(&mut self, dvels: &mut DVectorSliceMut<N>) {}

    #[inline]
    fn step_solve_internal_position_constraints(&mut self, params: &IntegrationParameters<N>) {}
}


impl<N: Real> BodyPart<N> for TetrahedralElement<N> {
    fn handle(&self) -> Option<BodyPartHandle> {
        self.handle
    }

    fn center_of_mass(&self) -> Point3<N> {
        self.com
    }

    fn position(&self) -> Isometry3<N> {
        Isometry3::new(self.com.coords, na::zero())
    }

    fn velocity(&self) -> Velocity<N> {
        unimplemented!()
    }

    fn inertia(&self) -> Inertia<N> {
        Inertia::new(self.volume * self.density, Matrix3::identity())
    }

    fn local_inertia(&self) -> Inertia<N> {
        Inertia::new(self.volume * self.density, Matrix3::identity())
    }

    fn apply_force(&mut self, force: &Force<N>) {
        unimplemented!()
    }
}