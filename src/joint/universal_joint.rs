use na::{self, DVectorSliceMut, Isometry3, RealField, Translation3, Unit, Vector3};

use crate::joint::{Joint, RevoluteJoint};
use crate::math::{JacobianSliceMut, Velocity};
use crate::object::{Multibody, MultibodyLink};
use crate::solver::{ConstraintSet, GenericNonlinearConstraint, IntegrationParameters};

/// A joint that allows only two relative rotations between two multibody links.
#[derive(Copy, Clone, Debug)]
pub struct UniversalJoint<N: RealField> {
    revo1: RevoluteJoint<N>,
    revo2: RevoluteJoint<N>,

    coupling_dot: Velocity<N>,
    coupling_dot_veldiff: Velocity<N>,
}

impl<N: RealField> UniversalJoint<N> {
    /// Creates an universal joint allowing rotations along `axis1` and `axis2`.
    ///
    /// The axii are expressed in the local coordinate systems of the attached multibody links.
    pub fn new(axis1: Unit<Vector3<N>>, axis2: Unit<Vector3<N>>, angle1: N, angle2: N) -> Self {
        UniversalJoint {
            revo1: RevoluteJoint::new(axis1, angle1),
            revo2: RevoluteJoint::new(axis2, angle2),
            coupling_dot: Velocity::zero(),
            coupling_dot_veldiff: Velocity::zero(),
        }
    }
}

impl<N: RealField> Joint<N> for UniversalJoint<N> {
    #[inline]
    fn clone(&self) -> Box<Joint<N>> {
        Box::new(*self)
    }

    #[inline]
    fn ndofs(&self) -> usize {
        2
    }

    fn body_to_parent(&self, parent_shift: &Vector3<N>, body_shift: &Vector3<N>) -> Isometry3<N> {
        Translation3::from(*parent_shift) * self.revo1.rotation()
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

    fn default_damping(&self, out: &mut DVectorSliceMut<N>) {
        self.revo1.default_damping(&mut out.rows_mut(0, 1));
        self.revo2.default_damping(&mut out.rows_mut(1, 1));
    }

    fn integrate(&mut self, params: &IntegrationParameters<N>, vels: &[N]) {
        self.revo1.integrate(params, vels);
        self.revo2.integrate(params, &[vels[1]]);
    }

    fn apply_displacement(&mut self, disp: &[N]) {
        self.revo1.apply_displacement(disp);
        self.revo2.apply_displacement(&[disp[1]]);
    }

    fn num_velocity_constraints(&self) -> usize {
        self.revo1.num_velocity_constraints() + self.revo2.num_velocity_constraints()
    }

    fn velocity_constraints(
        &self,
        params: &IntegrationParameters<N>,
        multibody: &Multibody<N>,
        link: &MultibodyLink<N>,
        assembly_id: usize,
        dof_id: usize,
        ext_vels: &[N],
        ground_j_id: &mut usize,
        jacobians: &mut [N],
        constraints: &mut ConstraintSet<N>,
    ) {
        self.revo1.velocity_constraints(
            params,
            multibody,
            link,
            assembly_id,
            dof_id,
            ext_vels,
            ground_j_id,
            jacobians,
            constraints,
        );
        self.revo2.velocity_constraints(
            params,
            multibody,
            link,
            assembly_id,
            dof_id + 1,
            ext_vels,
            ground_j_id,
            jacobians,
            constraints,
        );
    }

    fn num_position_constraints(&self) -> usize {
        // NOTE: we don't test if constraints exist to simplify indexing.
        2
    }

    fn position_constraint(
        &self,
        i: usize,
        multibody: &Multibody<N>,
        link: &MultibodyLink<N>,
        dof_id: usize,
        jacobians: &mut [N],
    ) -> Option<GenericNonlinearConstraint<N>> {
        if i == 0 {
            self.revo1.position_constraint(0, multibody, link, dof_id, jacobians)
        } else {
            self.revo2
                .position_constraint(0, multibody, link, dof_id + 1, jacobians)
        }
    }
}

revolute_motor_limit_methods_1!(UniversalJoint, revo1);
revolute_motor_limit_methods_2!(UniversalJoint, revo2);
