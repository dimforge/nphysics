use std::ops::AddAssign;
use std::iter;
use std::collections::HashMap;
use na::{self, Real, Point3, Point4, Vector3, Matrix3, DMatrix,
         DVector, DVectorSlice, DVectorSliceMut, LU, Dynamic, U3};
use ncollide::utils;
use ncollide::shape::{TriMesh, DeformationsType, DeformationIndex};

use object::{Body, BodyPart, BodyHandle, BodyPartHandle, BodyStatus, ActivationStatus};
use solver::IntegrationParameters;
use math::Force;

/// One element of a deformable volume.
pub struct TetrahedralElement<N: Real> {
    indices: Point4<usize>,
    com: Point3<N>,
    j: Matrix3<N>,
    j_inv: Matrix3<N>,
    // orientation: Rotation3<N>,
    volume: N,
    density: N,
}

/// A deformable volume using FEM to simulate linear elasticity.
///
/// The volume is described by a set of tetrahedral elements. This
/// implements an isoparametric approach where the interpolations are linear.
pub struct DeformableVolume<N: Real> {
    handle: Option<BodyHandle>,
    elements: Vec<TetrahedralElement<N>>,
    positions: DVector<N>,
    velocities: DVector<N>,
    accelerations: DVector<N>,
    damping: DMatrix<N>,
    stiffness: DMatrix<N>,
    augmented_mass: DMatrix<N>,
    inv_augmented_mass: LU<N, Dynamic, Dynamic>,

    // Cache.
    // FIXME: use a workspace common with other deformables?
    dpos: DVector<N>,

    // Constraints.
    rest_positions: DVector<N>,
    damping_coeffs: (N, N),
    young_modulus: N,
    poisson_ratio: N,

    companion_id: usize,
    activation: ActivationStatus<N>,
    status: BodyStatus,
}

impl<N: Real> DeformableVolume<N> {
    /// Initializes a new deformable volume from its tetrahedral elements.
    pub fn new(vertices: &Vec<Point3<N>>, tetrahedra: &Vec<Point4<usize>>, density: N, young_modulus: N, poisson_ratio: N, damping_coeffs: (N, N)) -> Self {
        let elements = tetrahedra.iter().map(|idx|
            TetrahedralElement {
                indices: idx * 3,
                com: Point3::origin(),
                j: na::zero(),
                j_inv: na::zero(),
                volume: na::zero(),
                density,
            }).collect();

        let ndofs = vertices.len() * 3;
        let rest_positions = DVector::from_iterator(ndofs, vertices.iter().flat_map(|p| p.iter().cloned()));

        DeformableVolume {
            handle: None,
            elements,
            positions: rest_positions.clone(),
            velocities: DVector::zeros(ndofs),
            accelerations: DVector::zeros(ndofs),
            damping: DMatrix::zeros(ndofs, ndofs),
            stiffness: DMatrix::zeros(ndofs, ndofs),
            augmented_mass: DMatrix::zeros(ndofs, ndofs),
            inv_augmented_mass: LU::new(DMatrix::zeros(0, 0)),
            dpos: DVector::zeros(ndofs),
            rest_positions,
            damping_coeffs,
            young_modulus,
            poisson_ratio,
            companion_id: 0,
            activation: ActivationStatus::new_active(),
            status: BodyStatus::Dynamic,
        }
    }

    /// The position of this body in generalized coordinates.
    #[inline]
    pub fn positions(&self) -> &DVector<N> {
        &self.positions
    }

    fn assemble_forces(&mut self, gravity: &Vector3<N>, params: &IntegrationParameters<N>) {
        for elt in &self.elements {
            let contribution = gravity * (elt.volume * na::convert::<_, N>(1.0 / 4.0));

            for k in 0..4 {
                let mut forces_part = self.accelerations.fixed_rows_mut::<U3>(elt.indices[k]);
                forces_part += contribution;
            }
        }

        // Compute: self.forces - self.stiffness * (pos - rest_pos + vel * dt) - self.damping * vel;
        self.dpos.copy_from(&self.positions);
        self.dpos.axpy(-N::one(), &self.rest_positions, N::one());
        self.dpos.axpy(params.dt, &self.velocities, N::one());
        self.accelerations.gemv(-N::one(), &self.stiffness, &self.dpos, N::one());
        self.accelerations.gemv(-N::one(), &self.damping, &self.velocities, N::one());
    }

    fn assemble_mass(&mut self) {
        for elt in &self.elements {
            let coeff_mass = elt.density * elt.volume / na::convert::<_, N>(20.0f64);

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

    fn assemble_damping(&mut self) {
        let d = self.damping.as_mut_slice();
        let m = self.augmented_mass.as_slice();
        let s = self.stiffness.as_slice();

        for i in 0..d.len() {
            d[i] = self.damping_coeffs.0 * m[i] + self.damping_coeffs.1 * s[i]
        }
    }

    fn assemble_stiffness(&mut self) {
        let _1: N = na::one();
        let _2: N = na::convert(2.0);
        let _6: N = na::convert(6.0);

        for elt in &self.elements {
            let d0 = (self.young_modulus * (_1 - self.poisson_ratio)) / ((_1 + self.poisson_ratio) * (_1 - _2 * self.poisson_ratio));
            let d1 = (self.young_modulus * self.poisson_ratio) / ((_1 + self.poisson_ratio) * (_1 - _2 * self.poisson_ratio));
            let d2 = (self.young_modulus * (_1 - _2 * self.poisson_ratio)) / (_2 * (_1 + self.poisson_ratio) * (_1 - _2 * self.poisson_ratio));

            for a in 0..4 {
                for b in 0..4 {
                    let bn;
                    let cn;
                    let dn;
                    let bm;
                    let cm;
                    let dm;

                    if a == 0 {
                        bn = -elt.j_inv.m11 - elt.j_inv.m12 - elt.j_inv.m13;
                        cn = -elt.j_inv.m21 - elt.j_inv.m22 - elt.j_inv.m23;
                        dn = -elt.j_inv.m31 - elt.j_inv.m32 - elt.j_inv.m33;
                    } else {
                        bn = elt.j_inv[(0, a - 1)];
                        cn = elt.j_inv[(1, a - 1)];
                        dn = elt.j_inv[(2, a - 1)];
                    }

                    if b == 0 {
                        bm = -elt.j_inv.m11 - elt.j_inv.m12 - elt.j_inv.m13;
                        cm = -elt.j_inv.m21 - elt.j_inv.m22 - elt.j_inv.m23;
                        dm = -elt.j_inv.m31 - elt.j_inv.m32 - elt.j_inv.m33;
                    } else {
                        bm = elt.j_inv[(0, b - 1)];
                        cm = elt.j_inv[(1, b - 1)];
                        dm = elt.j_inv[(2, b - 1)];
                    }

                    let node_stiffness = Matrix3::new(
                        d0 * bn * bm + d2 * (cn * cm + dn * dm), d1 * bn * cm + d2 * cn * bm, d1 * bn * dm + d2 * dn * bm,
                        d1 * cn * bm + d2 * bn * cm, d0 * cn * cm + d2 * (bn * bm + dn * dm), d1 * cn * dm + d2 * dn * cm,
                        d1 * dn * bm + d2 * bn * dm, d1 * dn * cm + d2 * cn * dm, d0 * dn * dm + d2 * (bn * bm + cn * cm),
                    ) * elt.volume;


                    let ia = elt.indices[a];
                    let ib = elt.indices[b];

                    // FIXME: meld it directly with the mass matrix?
                    self.stiffness.fixed_slice_mut::<U3, U3>(ia, ib).add_assign(&node_stiffness);
                }
            }
        }
    }

    /// Returns the triangles at the boundary of this volume.
    pub fn boundary(&self) -> Vec<Point3<usize>> {
        fn key(a: usize, b: usize, c: usize) -> (usize, usize, usize) {
            let (sa, sb, sc) = utils::sort3(&a, &b, &c);
            (*sa, *sb, *sc)
        }

        let mut faces = HashMap::new();

        for (i, elt) in self.elements.iter().enumerate() {
            let k1 = key(elt.indices.x, elt.indices.y, elt.indices.z);
            let k2 = key(elt.indices.y, elt.indices.z, elt.indices.w);
            let k3 = key(elt.indices.z, elt.indices.w, elt.indices.x);
            let k4 = key(elt.indices.w, elt.indices.x, elt.indices.y);

            faces.entry(k1).or_insert((0, elt.indices.w)).0.add_assign(1);
            faces.entry(k2).or_insert((0, elt.indices.x)).0.add_assign(1);
            faces.entry(k3).or_insert((0, elt.indices.y)).0.add_assign(1);
            faces.entry(k4).or_insert((0, elt.indices.z)).0.add_assign(1);
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
                    Some(Point3::new(k.0, k.1, k.2))
                } else {
                    Some(Point3::new(k.0, k.2, k.1))
                }
            } else {
                None
            }
        }).collect();

        boundary
    }

    /// Returns a triangle mesh at the boundary of this volume as well as a mapping between the mesh
    /// vertices and this volume degrees of freedom.
    pub fn boundary_mesh(&self) -> (TriMesh<N>, Vec<DeformationIndex>) {
        const INVALID: usize = usize::max_value();
        let mut deformation_indices = Vec::new();
        let mut indices = self.boundary();
        let mut idx_remap: Vec<usize> = iter::repeat(INVALID).take(self.positions.len() / 3).collect();
        let mut vertices = Vec::new();

        for idx in &mut indices {
            for i in 0..3 {
                let idx_i = &mut idx[i];
                if idx_remap[*idx_i / 3] == INVALID {
                    let new_id = vertices.len();
                    vertices.push(Point3::new(
                        self.positions[*idx_i + 0],
                        self.positions[*idx_i + 1],
                        self.positions[*idx_i + 2])
                    );
                    deformation_indices.push(DeformationIndex {
                        source: *idx_i,
                        target: new_id,
                    });
                    idx_remap[*idx_i / 3] = new_id;
                    *idx_i = new_id;
                } else {
                    *idx_i = idx_remap[*idx_i / 3];
                }
            }
        }

        (TriMesh::new(vertices, indices, None), deformation_indices)
    }

    /// Constructs an axis-aligned cube with regular subdivisions along each axis.
    ///
    /// The cube is subdivided `nx` (resp. `ny` and `nz`) times along
    /// the `x` (resp. `y` and `z`) axis.
    pub fn cube(extents: Vector3<N>, nx: usize, ny: usize, nz: usize, density: N, young_modulus: N, poisson_ratio: N, damping_coeffs: (N, N)) -> Self {
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
                    vertices.push(Point3::new(x * extents.x, y * extents.y, z * extents.z))
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
                    indices.push(Point4::new(_0, _1, _2, _5));
                    indices.push(Point4::new(_2, _5, _6, _7));
                    indices.push(Point4::new(_2, _7, _3, _0));
                    indices.push(Point4::new(_7, _4, _0, _5));
                    indices.push(Point4::new(_0, _2, _7, _5));
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
            elt.volume = elt.j.determinant() * na::convert(1.0 / 6.0);
            elt.j_inv = elt.j.try_inverse().expect("Degenerate tetrahedral element found.");
            elt.com = Point3::from_coordinates((a + b + c + d) * na::convert::<_, N>(1.0 / 4.0));
// FIXME: update orientation for stiffness wrapping.
        }
    }

    /// Update the dynamics property of this deformable volume.
    fn update_dynamics(&mut self,
                       gravity: &Vector3<N>,
                       params: &IntegrationParameters<N>) {
        self.assemble_stiffness();
        self.assemble_mass();
        self.assemble_damping();
        self.assemble_forces(gravity, params);

// Finalize assembling the augmented mass.
        {
            let dt2 = params.dt * params.dt;
            let d = self.damping.as_slice();
            let m = self.augmented_mass.as_mut_slice();
            let s = self.stiffness.as_slice();

            for i in 0..d.len() {
                m[i] += params.dt * d[i] + dt2 * s[i];
            }
        }

        // FIXME: avoid allocation inside LU at each timestep.
        self.inv_augmented_mass = LU::new(self.augmented_mass.clone());
        assert!(self.inv_augmented_mass.solve_mut(&mut self.accelerations));
    }

    fn clear_dynamics(&mut self) {
        self.augmented_mass.fill(N::zero());
        self.stiffness.fill(N::zero());
        self.accelerations.fill(N::zero());
    }

    fn apply_displacement(&mut self, disp: &[N]) {
        let disp = DVectorSlice::from_slice(disp, self.positions.len());
        self.positions += disp;
    }

    fn set_handle(&mut self, handle: Option<BodyHandle>) {
        self.handle = handle;
    }

    fn handle(&self) -> Option<BodyHandle> {
        self.handle
    }

    fn status(&self) -> BodyStatus {
        self.status
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
        for v in &mut self.velocities {
            *v = N::zero()
        }
    }

    fn part(&self, handle: BodyPartHandle) -> &BodyPart<N> {
        unimplemented!()
    }

    fn part_mut(&mut self, handle: BodyPartHandle) -> &mut BodyPart<N> {
        unimplemented!()
    }

    fn contains_part(&self, handle: BodyPartHandle) -> bool {
        unimplemented!()
    }

    fn inv_mass_mul_generalized_forces(&self, out: &mut [N]) {
        unimplemented!()
    }

    fn body_part_jacobian_mul_force(&self, part: &BodyPart<N>, force: &Force<N>, out: &mut [N]) {
        unimplemented!()
    }

    fn inv_mass_mul_body_part_force(&self, part: &BodyPart<N>, force: &Force<N>, out: &mut [N]) {
        unimplemented!()
    }
}

/*
impl<N: Real> BodyPart<N> for TetrahedralElement<N> {
    fn handle(&self) -> Option<BodyPartHandle>;

    fn center_of_mass(&self) -> Point<N>;

    fn position(&self) -> Isometry<N>;

    fn velocity(&self) -> Velocity<N>;

    fn inertia(&self) -> Inertia<N>;

    fn local_inertia(&self) -> Inertia<N>;

    fn apply_force(&mut self, force: &Force<N>);
}
*/