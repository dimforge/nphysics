use na::{DVectorSliceMut, Isometry3, RealField, Unit, Vector3};

use crate::joint::{Joint, PrismaticJoint, RevoluteJoint};
use crate::solver::{ConstraintSet, GenericNonlinearConstraint, IntegrationParameters};
use crate::object::{Multibody, MultibodyLink};
use crate::math::{JacobianSliceMut, Velocity};

/// A joint that allows one translational and one rotational degrees of freedom along a single axis.
#[derive(Copy, Clone, Debug)]
pub struct CylindricalJoint<N: RealField> {
    prism: PrismaticJoint<N>,
    revo: RevoluteJoint<N>,
}

impl<N: RealField> CylindricalJoint<N> {
    /// Create a cylindrical joint with the given axis and initial position of angle.
    /// 
    /// The axis is expressed in the local space of the multibody links attached to this joint.
    pub fn new(axis: Unit<Vector3<N>>, position: N, angle: N) -> Self {
        let prism = PrismaticJoint::new(axis, position);
        let revo = RevoluteJoint::new(axis, angle);

        CylindricalJoint { prism, revo }
    }
}

impl<N: RealField> Joint<N> for CylindricalJoint<N> {
    #[inline]
    fn clone(&self) -> Box<Joint<N>> {
        Box::new(*self)
    }

    #[inline]
    fn ndofs(&self) -> usize {
        2
    }

    fn body_to_parent(&self, parent_shift: &Vector3<N>, body_shift: &Vector3<N>) -> Isometry3<N> {
        self.prism.translation() * self.revo.body_to_parent(parent_shift, body_shift)
    }

    fn update_jacobians(&mut self, body_shift: &Vector3<N>, vels: &[N]) {
        self.prism.update_jacobians(body_shift, vels);
        self.revo.update_jacobians(body_shift, &[vels[1]]);
    }

    fn jacobian(&self, transform: &Isometry3<N>, out: &mut JacobianSliceMut<N>) {
        self.prism.jacobian(transform, &mut out.columns_mut(0, 1));
        self.revo.jacobian(transform, &mut out.columns_mut(1, 1));
    }

    fn jacobian_dot(&self, transform: &Isometry3<N>, out: &mut JacobianSliceMut<N>) {
        self.prism
            .jacobian_dot(transform, &mut out.columns_mut(0, 1));
        self.revo
            .jacobian_dot(transform, &mut out.columns_mut(1, 1));
    }

    fn jacobian_dot_veldiff_mul_coordinates(
        &self,
        transform: &Isometry3<N>,
        vels: &[N],
        out: &mut JacobianSliceMut<N>,
    ) {
        self.prism.jacobian_dot_veldiff_mul_coordinates(
            transform,
            vels,
            &mut out.columns_mut(0, 1),
        );
        self.revo.jacobian_dot_veldiff_mul_coordinates(
            transform,
            &[vels[1]],
            &mut out.columns_mut(1, 1),
        );
    }

    fn jacobian_mul_coordinates(&self, vels: &[N]) -> Velocity<N> {
        self.prism.jacobian_mul_coordinates(vels) + self.revo.jacobian_mul_coordinates(&[vels[1]])
    }

    fn jacobian_dot_mul_coordinates(&self, vels: &[N]) -> Velocity<N> {
        // NOTE: The following is zero.
        // self.prism.jacobian_dot_mul_coordinates(vels) +
        self.revo.jacobian_dot_mul_coordinates(&[vels[1]])
    }

    fn default_damping(&self, out: &mut DVectorSliceMut<N>) {
        self.prism.default_damping(&mut out.rows_mut(0, 1));
        self.revo.default_damping(&mut out.rows_mut(1, 1));
    }

    fn integrate(&mut self, params: &IntegrationParameters<N>, vels: &[N]) {
        self.prism.integrate(params, vels);
        self.revo.integrate(params, &[vels[1]]);
    }

    fn apply_displacement(&mut self, disp: &[N]) {
        self.prism.apply_displacement(disp);
        self.revo.apply_displacement(&[disp[1]]);
    }

    fn num_velocity_constraints(&self) -> usize {
        self.prism.num_velocity_constraints() + self.revo.num_velocity_constraints()
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
        self.prism.velocity_constraints(
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
        self.revo.velocity_constraints(
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
            self.prism.position_constraint(0, multibody, link, dof_id, jacobians)
        } else {
            self.revo
                .position_constraint(0, multibody, link, dof_id + 1, jacobians)
        }
    }
}

prismatic_motor_limit_methods!(CylindricalJoint, prism);
revolute_motor_limit_methods!(CylindricalJoint, revo);
