use na::{self, Isometry3, Real, Translation3, Unit, Vector3};

use joint::{Joint, PrismaticJoint};
use solver::{ConstraintSet, GenericNonlinearConstraint, IntegrationParameters};
use object::MultibodyLinkRef;
use math::{JacobianSliceMut, Velocity};

#[derive(Copy, Clone, Debug)]
pub struct RectangularJoint<N: Real> {
    prism1: PrismaticJoint<N>,
    prism2: PrismaticJoint<N>,
}

impl<N: Real> RectangularJoint<N> {
    pub fn new(axis1: Unit<Vector3<N>>, axis2: Unit<Vector3<N>>, offset1: N, offset2: N) -> Self {
        RectangularJoint {
            prism1: PrismaticJoint::new(axis1, offset1),
            prism2: PrismaticJoint::new(axis2, offset2),
        }
    }
}

impl<N: Real> Joint<N> for RectangularJoint<N> {
    #[inline]
    fn ndofs(&self) -> usize {
        2
    }

    fn body_to_parent(&self, parent_shift: &Vector3<N>, body_shift: &Vector3<N>) -> Isometry3<N> {
        let t = Translation3::from_vector(parent_shift - body_shift) * self.prism1.translation()
            * self.prism2.translation();
        Isometry3::from_parts(t, na::one())
    }

    fn update_jacobians(&mut self, body_shift: &Vector3<N>, vels: &[N]) {
        self.prism1.update_jacobians(body_shift, vels);
        self.prism2.update_jacobians(body_shift, &[vels[1]]);
    }

    fn jacobian(&self, transform: &Isometry3<N>, out: &mut JacobianSliceMut<N>) {
        self.prism1.jacobian(transform, &mut out.columns_mut(0, 1));
        self.prism2.jacobian(transform, &mut out.columns_mut(1, 1));
    }

    fn jacobian_dot(&self, _: &Isometry3<N>, _: &mut JacobianSliceMut<N>) {}

    fn jacobian_dot_veldiff_mul_coordinates(
        &self,
        _: &Isometry3<N>,
        _: &[N],
        _: &mut JacobianSliceMut<N>,
    ) {

    }

    fn jacobian_mul_coordinates(&self, vels: &[N]) -> Velocity<N> {
        self.prism1.jacobian_mul_coordinates(vels)
            + self.prism2.jacobian_mul_coordinates(&[vels[1]])
    }

    fn jacobian_dot_mul_coordinates(&self, _: &[N]) -> Velocity<N> {
        Velocity::zero()
    }

    fn integrate(&mut self, params: &IntegrationParameters<N>, vels: &[N]) {
        self.prism1.integrate(params, vels);
        self.prism2.integrate(params, &[vels[1]]);
    }

    fn apply_displacement(&mut self, disp: &[N]) {
        self.prism1.apply_displacement(disp);
        self.prism2.apply_displacement(&[disp[1]]);
    }

    fn nconstraints(&self) -> usize {
        self.prism1.nconstraints() + self.prism2.nconstraints()
    }

    fn build_constraints(
        &self,
        params: &IntegrationParameters<N>,
        link: &MultibodyLinkRef<N>,
        assembly_id: usize,
        dof_id: usize,
        ext_vels: &[N],
        ground_jacobian_id: &mut usize,
        jacobians: &mut [N],
        constraints: &mut ConstraintSet<N>,
    ) {
        self.prism1.build_constraints(
            params,
            link,
            assembly_id,
            dof_id,
            ext_vels,
            ground_jacobian_id,
            jacobians,
            constraints,
        );
        self.prism2.build_constraints(
            params,
            link,
            assembly_id,
            dof_id + 1,
            ext_vels,
            ground_jacobian_id,
            jacobians,
            constraints,
        );
    }

    fn nposition_constraints(&self) -> usize {
        // NOTE: we don't test if constraints exist to simplify indexing.
        2
    }

    fn position_constraint(
        &self,
        i: usize,
        link: &MultibodyLinkRef<N>,
        dof_id: usize,
        jacobians: &mut [N],
    ) -> Option<GenericNonlinearConstraint<N>> {
        if i == 0 {
            self.prism1.position_constraint(0, link, dof_id, jacobians)
        } else {
            self.prism2
                .position_constraint(0, link, dof_id + 1, jacobians)
        }
    }
}

prismatic_motor_limit_methods_1!(RectangularJoint, prism1);
prismatic_motor_limit_methods_2!(RectangularJoint, prism2);
