use na::{DVectorSliceMut, Isometry3, RealField, Unit, Vector3};

use crate::joint::{Joint, PrismaticJoint, RevoluteJoint};
use crate::math::{JacobianSliceMut, Velocity};
use crate::object::{Multibody, MultibodyLink};
use crate::solver::{ConstraintSet, GenericNonlinearConstraint, IntegrationParameters};

/// A joint that allows 1 rotational and 2 translational degrees of freedom.
#[derive(Copy, Clone, Debug)]
pub struct PlanarJoint<N: RealField> {
    prism1: PrismaticJoint<N>,
    prism2: PrismaticJoint<N>,
    revo: RevoluteJoint<N>,
}

impl<N: RealField> PlanarJoint<N> {
    /// Create a new planar joint where both translational degrees of freedoms are along the provide axii.
    ///
    /// The rotational degree of freedom is along an axis orthogonal to `axis1` and `axis2`. Idealy, the two
    /// provided axii should be orthogonal. All axis are in the local coordinate space of the attached multibody links.
    ///
    /// Panics if `axis1` and `axis2` are near-colinear.
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

impl<N: RealField> Joint<N> for PlanarJoint<N> {
    #[inline]
    fn clone(&self) -> Box<Joint<N>> {
        Box::new(*self)
    }

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

    fn default_damping(&self, out: &mut DVectorSliceMut<N>) {
        self.prism1.default_damping(&mut out.rows_mut(0, 1));
        self.prism2.default_damping(&mut out.rows_mut(1, 1));
        self.revo.default_damping(&mut out.rows_mut(2, 1));
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

    fn num_velocity_constraints(&self) -> usize {
        self.prism1.num_velocity_constraints() + self.prism2.num_velocity_constraints()
            + self.revo.num_velocity_constraints()
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
        self.prism1.velocity_constraints(
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
        self.prism2.velocity_constraints(
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
        self.revo.velocity_constraints(
            params,
            multibody,
            link,
            assembly_id,
            dof_id + 2,
            ext_vels,
            ground_j_id,
            jacobians,
            constraints,
        );
    }

    fn num_position_constraints(&self) -> usize {
        // NOTE: we don't test if constraints exist to simplify indexing.
        3
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
            self.prism1.position_constraint(0, multibody, link, dof_id, jacobians)
        } else if i == 1 {
            self.prism2
                .position_constraint(0, multibody, link, dof_id + 1, jacobians)
        } else {
            self.revo
                .position_constraint(0, multibody, link, dof_id + 2, jacobians)
        }
    }
}

prismatic_motor_limit_methods_1!(PlanarJoint, prism1);
prismatic_motor_limit_methods_2!(PlanarJoint, prism2);
revolute_motor_limit_methods!(PlanarJoint, revo);
