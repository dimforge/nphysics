use na::{Isometry3, Real, Translation3, Unit, Vector3};

use joint::{Joint, JointMotor, RevoluteJoint, UnitJoint};
use solver::{BilateralGroundConstraint, ConstraintSet, IntegrationParameters,
             UnilateralGroundConstraint};
use object::{Multibody, MultibodyLinkRef};
use math::{JacobianSliceMut, Velocity};

#[derive(Copy, Clone, Debug)]
pub struct HelicalJoint<N: Real> {
    revo: RevoluteJoint<N>,
    pitch: N,
}

impl<N: Real> HelicalJoint<N> {
    pub fn new(axis: Unit<Vector3<N>>, pitch: N, angle: N) -> Self {
        HelicalJoint {
            revo: RevoluteJoint::new(axis, angle),
            pitch: pitch,
        }
    }

    pub fn offset(&self) -> N {
        self.revo.angle() * self.pitch
    }

    pub fn angle(&self) -> N {
        self.revo.angle()
    }
}

impl<N: Real> Joint<N> for HelicalJoint<N> {
    #[inline]
    fn ndofs(&self) -> usize {
        1
    }

    fn body_to_parent(&self, parent_shift: &Vector3<N>, body_shift: &Vector3<N>) -> Isometry3<N> {
        Translation3::from_vector(self.revo.axis().as_ref() * self.revo.angle())
            * self.revo.body_to_parent(parent_shift, body_shift)
    }

    fn update_jacobians(&mut self, body_shift: &Vector3<N>, vels: &[N]) {
        self.revo.update_jacobians(body_shift, vels)
    }

    fn jacobian(&self, transform: &Isometry3<N>, out: &mut JacobianSliceMut<N>) {
        let mut jac = *self.revo.local_jacobian();
        jac.linear += self.revo.axis().as_ref() * self.pitch;
        out.copy_from(jac.transformed(transform).as_vector())
    }

    fn jacobian_dot(&self, transform: &Isometry3<N>, out: &mut JacobianSliceMut<N>) {
        self.revo.jacobian_dot(transform, out)
    }

    fn jacobian_dot_veldiff_mul_coordinates(
        &self,
        transform: &Isometry3<N>,
        acc: &[N],
        out: &mut JacobianSliceMut<N>,
    ) {
        self.revo
            .jacobian_dot_veldiff_mul_coordinates(transform, acc, out)
    }

    fn jacobian_mul_coordinates(&self, vels: &[N]) -> Velocity<N> {
        let mut jac = *self.revo.local_jacobian();
        jac.linear += self.revo.axis().as_ref() * self.pitch;
        jac * vels[0]
    }

    fn jacobian_dot_mul_coordinates(&self, vels: &[N]) -> Velocity<N> {
        self.revo.jacobian_dot_mul_coordinates(vels)
    }

    fn apply_displacement(&mut self, params: &IntegrationParameters<N>, vels: &[N]) {
        self.revo.apply_displacement(params, vels)
    }

    fn nconstraints(&self) -> usize {
        self.revo.nconstraints()
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
        self.revo.build_constraints(
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
    }
}

impl<N: Real> UnitJoint<N> for HelicalJoint<N> {
    fn position(&self) -> N {
        self.revo.angle()
    }

    fn motor(&self) -> &JointMotor<N, N> {
        self.revo.motor()
    }

    fn min_position(&self) -> Option<N> {
        self.revo.min_angle()
    }

    fn max_position(&self) -> Option<N> {
        self.revo.max_angle()
    }
}

revolute_motor_limit_methods!(HelicalJoint, revo);
