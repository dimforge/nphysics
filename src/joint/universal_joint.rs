use na::{self, Isometry3, Real, Translation3, Unit, Vector3};

use joint::{Joint, RevoluteJoint};
use solver::{BilateralGroundConstraint, ConstraintSet, IntegrationParameters,
             UnilateralGroundConstraint};
use object::{Multibody, MultibodyLinkRef};
use math::{JacobianSliceMut, Velocity};

#[derive(Copy, Clone, Debug)]
pub struct UniversalJoint<N: Real> {
    revo1: RevoluteJoint<N>,
    revo2: RevoluteJoint<N>,

    coupling_dot: Velocity<N>,
    coupling_dot_veldiff: Velocity<N>,
}

impl<N: Real> UniversalJoint<N> {
    pub fn new(axis1: Unit<Vector3<N>>, axis2: Unit<Vector3<N>>, angle1: N, angle2: N) -> Self {
        UniversalJoint {
            revo1: RevoluteJoint::new(axis1, angle1),
            revo2: RevoluteJoint::new(axis2, angle2),
            coupling_dot: Velocity::zero(),
            coupling_dot_veldiff: Velocity::zero(),
        }
    }
}

impl<N: Real> Joint<N> for UniversalJoint<N> {
    #[inline]
    fn ndofs(&self) -> usize {
        2
    }

    fn body_to_parent(&self, parent_shift: &Vector3<N>, body_shift: &Vector3<N>) -> Isometry3<N> {
        Translation3::from_vector(*parent_shift) * self.revo1.rotation()
            * self.revo2.body_to_parent(&na::zero(), body_shift)
    }

    fn update_jacobians(&mut self, body_shift: &Vector3<N>, vels: &[N]) {
        self.revo1.update_jacobians(body_shift, vels);
        self.revo2.update_jacobians(body_shift, &[vels[1]]);

        let axis1 = self.revo1.axis();
        let rot1 = self.revo1.rotation();
        let jac2 = self.revo2.local_jacobian().rotated(rot1);

        self.coupling_dot_veldiff =
            Velocity::new(axis1.cross(&jac2.linear), axis1.cross(&jac2.angular));
        self.coupling_dot = self.coupling_dot_veldiff * vels[0];
    }

    fn jacobian(&self, transform: &Isometry3<N>, out: &mut JacobianSliceMut<N>) {
        self.revo1.jacobian(transform, &mut out.columns_mut(0, 1));
        self.revo2.jacobian(
            &(transform * self.revo1.rotation()),
            &mut out.columns_mut(1, 1),
        );
    }

    fn jacobian_dot(&self, transform: &Isometry3<N>, out: &mut JacobianSliceMut<N>) {
        self.revo1
            .jacobian_dot(transform, &mut out.columns_mut(0, 1));
        let rot1 = self.revo1.rotation();
        let jac2 = self.revo2.local_jacobian_dot().rotated(rot1) + self.coupling_dot;
        out.column_mut(1)
            .copy_from(jac2.transformed(transform).as_vector())
    }

    fn jacobian_dot_veldiff_mul_coordinates(
        &self,
        transform: &Isometry3<N>,
        vels: &[N],
        out: &mut JacobianSliceMut<N>,
    ) {
        let jac1 = *self.revo1.local_jacobian_dot_veldiff() + self.coupling_dot_veldiff;
        out.column_mut(0)
            .copy_from(jac1.transformed(transform).as_vector());

        let rot1 = self.revo1.rotation();
        self.revo2.jacobian_dot_veldiff_mul_coordinates(
            &(transform * rot1),
            vels,
            &mut out.columns_mut(1, 1),
        );
    }

    fn jacobian_mul_coordinates(&self, vels: &[N]) -> Velocity<N> {
        self.revo1.jacobian_mul_coordinates(vels) + self.revo2.jacobian_mul_coordinates(&[vels[1]])
    }

    fn jacobian_dot_mul_coordinates(&self, vels: &[N]) -> Velocity<N> {
        let rot1 = self.revo1.rotation();
        self.revo1.jacobian_dot_mul_coordinates(vels)
            + self.revo2
                .jacobian_dot_mul_coordinates(&[vels[1]])
                .rotated(&rot1) + self.coupling_dot * vels[1]
    }

    fn apply_displacement(&mut self, params: &IntegrationParameters<N>, vels: &[N]) {
        self.revo1.apply_displacement(params, vels);
        self.revo2.apply_displacement(params, &[vels[1]]);
    }

    fn nconstraints(&self) -> usize {
        self.revo1.nconstraints() + self.revo2.nconstraints()
    }

    fn build_constraints(
        &self,
        params: &IntegrationParameters<N>,
        mb: &Multibody<N>,
        link: &MultibodyLinkRef<N>,
        assembly_id: usize,
        dof_id: usize,
        ext_vels: &[N],
        ground_jacobian_id: &mut usize,
        jacobians: &mut [N],
        constraints: &mut ConstraintSet<N>,
    ) {
        self.revo1.build_constraints(
            params,
            mb,
            link,
            assembly_id,
            dof_id,
            ext_vels,
            ground_jacobian_id,
            jacobians,
            constraints,
        );
        self.revo2.build_constraints(
            params,
            mb,
            link,
            assembly_id,
            dof_id + 1,
            ext_vels,
            ground_jacobian_id,
            jacobians,
            constraints,
        );
    }
}

revolute_motor_limit_methods_1!(UniversalJoint, revo1);
revolute_motor_limit_methods_2!(UniversalJoint, revo2);
