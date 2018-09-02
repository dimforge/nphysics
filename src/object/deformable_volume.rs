use std::ops::AddAssign;
use na::{self, Real, Point3, Point4, Vector3, Matrix3, DMatrix,
         DVector, DVectorSlice, DVectorSliceMut, LU, Dynamic, U3};

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
/// implements an isoparametric approach where the intepolation is linear.
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
                indices: *idx,
                com: Point3::origin(),
                j: na::zero(),
                j_inv: na::zero(),
                volume: na::zero(),
                density,
            }).collect();

        let ndofs = vertices.len() * 3;
        let rest_positions = DVector::from_iterator(ndofs, vertices.iter().flat_map(|p| p.iter().cloned()));

        DeformableVolume
            {
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

    /// Update the dynamics property of this deformable volume.
    pub fn update_dynamics(&mut self,
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
    }

    fn assemble_forces(&mut self, gravity: &Vector3<N>, params: &IntegrationParameters<N>) {
        for elt in &self.elements {
            let contribution = gravity * (elt.volume * na::convert::<_, N>(1.0 / 4.0));

            for k in 0..4 {
                let mut forces_part = self.accelerations.fixed_rows_mut::<U3>(elt.indices[k] * 3);
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

                    let ia = elt.indices[a] * 3;
                    let ib = elt.indices[b] * 3;

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


                    let ia = elt.indices[a] * 3;
                    let ib = elt.indices[b] * 3;

                    // FIXME: meld it directly with the mass matrix?
                    self.stiffness.fixed_slice_mut::<U3, U3>(ia, ib).add_assign(&node_stiffness);
                }
            }
        }
    }
}

impl<N: Real> Body<N> for DeformableVolume<N> {
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