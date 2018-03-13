use na::{Isometry3, Real, Unit, Vector3};

use joint::{Joint, PrismaticJoint, RevoluteJoint};
use solver::{ConstraintSet, GenericNonlinearConstraint, IntegrationParameters};
use object::MultibodyLinkRef;
use math::{JacobianSliceMut, Velocity};

#[derive(Copy, Clone, Debug)]
pub struct PlanarJoint<N: Real> {
    prism1: PrismaticJoint<N>,
    prism2: PrismaticJoint<N>,
    revo: RevoluteJoint<N>,
}

impl<N: Real> PlanarJoint<N> {
    pub fn new(
        axis1: Unit<Vector3<N>>,
        axis2: Unit<Vector3<N>>,
        pos1: N,
        pos2: N,
        angle: N,
    ) -> Self {
        let cross = axis1.cross(&*axis2);
        let normal = Unit::try_new(cross, N::default_epsilon())
            .expect("A planar joint cannot be defined from two collinear axis.");
        let prism1 = PrismaticJoint::new(axis1, pos1);
        let prism2 = PrismaticJoint::new(axis2, pos2);
        let revo = RevoluteJoint::new(normal, angle);

        PlanarJoint {
            prism1,
            prism2,
            revo,
        }
    }
}

impl<N: Real> Joint<N> for PlanarJoint<N> {
    #[inline]
    fn ndofs(&self) -> usize {
        3
    }

    fn body_to_parent(&self, parent_shift: &Vector3<N>, body_shift: &Vector3<N>) -> Isometry3<N> {
        self.prism1.translation() * self.prism2.translation()
            * self.revo.body_to_parent(parent_shift, body_shift)
    }

    fn update_jacobians(&mut self, body_shift: &Vector3<N>, vels: &[N]) {
        self.prism1.update_jacobians(body_shift, vels);
        self.prism2.update_jacobians(body_shift, &vels[1..]);
        self.revo.update_jacobians(body_shift, &vels[2..]);
    }

    fn jacobian(&self, transform: &Isometry3<N>, out: &mut JacobianSliceMut<N>) {
        self.prism1.jacobian(transform, &mut out.columns_mut(0, 1));
        self.prism2.jacobian(transform, &mut out.columns_mut(1, 1));
        self.revo.jacobian(transform, &mut out.columns_mut(2, 1));
    }

    fn jacobian_dot(&self, transform: &Isometry3<N>, out: &mut JacobianSliceMut<N>) {
        self.prism1
            .jacobian_dot(transform, &mut out.columns_mut(0, 1));
        self.prism2
            .jacobian_dot(transform, &mut out.columns_mut(1, 1));
        self.revo
            .jacobian_dot(transform, &mut out.columns_mut(2, 1));
    }

    fn jacobian_dot_veldiff_mul_coordinates(
        &self,
        transform: &Isometry3<N>,
        vels: &[N],
        out: &mut JacobianSliceMut<N>,
    ) {
        self.prism1.jacobian_dot_veldiff_mul_coordinates(
            transform,
            vels,
            &mut out.columns_mut(0, 1),
        );
        self.prism2.jacobian_dot_veldiff_mul_coordinates(
            transform,
            &[vels[1]],
            &mut out.columns_mut(1, 1),
        );
        self.revo.jacobian_dot_veldiff_mul_coordinates(
            transform,
            &[vels[2]],
            &mut out.columns_mut(2, 1),
        );
    }

    fn jacobian_mul_coordinates(&self, vels: &[N]) -> Velocity<N> {
        self.prism1.jacobian_mul_coordinates(vels)
            + self.prism2.jacobian_mul_coordinates(&[vels[1]])
            + self.revo.jacobian_mul_coordinates(&[vels[2]])
    }

    fn jacobian_dot_mul_coordinates(&self, vels: &[N]) -> Velocity<N> {
        // NOTE: The two folowing are zero.
        // self.prism1.jacobian_dot_mul_coordinates(vels)       +
        // self.prism2.jacobian_dot_mul_coordinates(&[vels[1]]) +
        self.revo.jacobian_dot_mul_coordinates(&[vels[2]])
    }

    fn integrate(&mut self, params: &IntegrationParameters<N>, vels: &[N]) {
        self.prism1.integrate(params, vels);
        self.prism2.integrate(params, &[vels[1]]);
        self.revo.integrate(params, &[vels[2]]);
    }

    fn apply_displacement(&mut self, disp: &[N]) {
        self.prism1.apply_displacement(disp);
        self.prism2.apply_displacement(&[disp[1]]);
        self.revo.apply_displacement(&[disp[2]]);
    }

    fn nconstraints(&self) -> usize {
        self.prism1.nconstraints() + self.prism2.nconstraints() + self.revo.nconstraints()
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
        self.revo.build_constraints(
            params,
            link,
            assembly_id,
            dof_id + 2,
            ext_vels,
            ground_jacobian_id,
            jacobians,
            constraints,
        );
    }

    fn nposition_constraints(&self) -> usize {
        // NOTE: we don't test if constraints exist to simplify indexing.
        3
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
        } else if i == 1 {
            self.prism2
                .position_constraint(0, link, dof_id + 1, jacobians)
        } else {
            self.revo
                .position_constraint(0, link, dof_id + 2, jacobians)
        }
    }
}

prismatic_motor_limit_methods_1!(PlanarJoint, prism1);
prismatic_motor_limit_methods_2!(PlanarJoint, prism2);
revolute_motor_limit_methods!(PlanarJoint, revo);
