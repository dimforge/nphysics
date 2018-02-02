use downcast::Any;

use na::{DVectorSliceMut, Real};

use joint::JointMotor;
use solver::{BilateralGroundConstraint, IntegrationParameters, UnilateralGroundConstraint};
use object::{Multibody, MultibodyLinkRef};
use math::{Isometry, JacobianSliceMut, Vector, Velocity};

pub trait Joint<N: Real>: Any + Send + Sync {
    fn ndofs(&self) -> usize;
    fn body_to_parent(&self, parent_shift: &Vector<N>, body_shift: &Vector<N>) -> Isometry<N>;
    fn update_jacobians(&mut self, body_shift: &Vector<N>, vels: &[N]);
    fn apply_displacement(&mut self, params: &IntegrationParameters<N>, vels: &[N]);

    // FIXME: rename those "copy_jacobian_to" ?
    fn jacobian(&self, transform: &Isometry<N>, out: &mut JacobianSliceMut<N>);
    fn jacobian_dot(&self, transform: &Isometry<N>, out: &mut JacobianSliceMut<N>);
    fn jacobian_dot_veldiff_mul_coordinates(
        &self,
        transform: &Isometry<N>,
        vels: &[N],
        out: &mut JacobianSliceMut<N>,
    );

    fn jacobian_mul_coordinates(&self, vels: &[N]) -> Velocity<N>;
    fn jacobian_dot_mul_coordinates(&self, vels: &[N]) -> Velocity<N>;

    fn nconstraints(&self) -> usize {
        0
    }

    fn build_constraints(
        &self,
        _params: &IntegrationParameters<N>,
        _mb: &Multibody<N>,
        _link: &MultibodyLinkRef<N>,
        _assembly_id: usize,
        _dof_id: usize,
        _ext_vels: &[N],
        _ground_jacobian_id: &mut usize,
        _jacobians: &mut [N],
        _out_unilateral: &mut Vec<UnilateralGroundConstraint<N>>,
        _out_bilateral: &mut Vec<BilateralGroundConstraint<N>>,
    ) {
    }
}

downcast!(<N> Joint<N> where N: Real);

pub trait UnitJoint<N: Real>: Joint<N> {
    fn position(&self) -> N;
    fn motor(&self) -> &JointMotor<N, N>;
    fn min_position(&self) -> Option<N>;
    fn max_position(&self) -> Option<N>;

    /*
     * fn jacobian(&self, transform: &Isometry<N>) -> Velocity<N>;
     * fn jacobian_dot(&self, transform: &Isometry<N>) -> Velocity<N>;
     * fn jacobian_dot_veldiff_mul_coordinates(&self, transform: &Isometry<N>) -> Velocity<N>;
     */
}

downcast!(<N> UnitJoint<N> where N: Real);

pub fn unit_joint_nconstraints<N: Real, J: UnitJoint<N>>(joint: &J) -> usize {
    // FIXME: don't always keep the constraints active.
    let mut nconstraints = 0;

    if joint.motor().enabled {
        nconstraints += 1;
    }
    if joint.min_position().is_some() {
        nconstraints += 1;
    }
    if joint.max_position().is_some() {
        nconstraints += 1;
    }

    nconstraints
}

pub fn build_unit_joint_constraints<N: Real, J: UnitJoint<N>>(
    joint: &J,
    params: &IntegrationParameters<N>,
    mb: &Multibody<N>,
    link: &MultibodyLinkRef<N>,
    assembly_id: usize,
    dof_id: usize,
    ext_vels: &[N],
    ground_jacobian_id: &mut usize,
    jacobians: &mut [N],
    out_unilateral: &mut Vec<UnilateralGroundConstraint<N>>,
    out_bilateral: &mut Vec<BilateralGroundConstraint<N>>,
) {
    let mut is_min_constraint_active = false;

    if joint.motor().enabled {
        let dvel = link.joint_velocity()[dof_id] + ext_vels[assembly_id + link.assembly_id()];

        DVectorSliceMut::new(&mut jacobians[*ground_jacobian_id..], mb.ndofs()).fill(N::zero());
        jacobians[*ground_jacobian_id + link.assembly_id() + dof_id] = N::one();

        let weighted_jacobian_id = *ground_jacobian_id + mb.ndofs();
        link.inv_mass_mul_unit_joint_force(
            dof_id,
            N::one(),
            &mut jacobians[weighted_jacobian_id..],
        );

        let inv_r = jacobians[weighted_jacobian_id + link.assembly_id() + dof_id]; // = J^t * M^-1 J
        let rhs = dvel - joint.motor().desired_velocity;
        let limits = joint.motor().impulse_limits();

        let constraint = BilateralGroundConstraint {
            impulse: N::zero(),
            r: N::one() / inv_r,
            rhs: rhs,
            limits: limits,
            assembly_id: assembly_id,
            jacobian_id: *ground_jacobian_id,
            weighted_jacobian_id: *ground_jacobian_id + mb.ndofs(),
            ndofs: mb.ndofs(),
        };

        out_bilateral.push(constraint);
        *ground_jacobian_id += 2 * mb.ndofs();
    }

    if let Some(min_position) = joint.min_position() {
        let err = min_position - joint.position();
        let dvel =
            link.joint_velocity()[dof_id] + ext_vels[assembly_id + link.assembly_id() + dof_id];

        if err >= N::zero() || err <= dvel * params.dt {
            is_min_constraint_active = true;
            DVectorSliceMut::new(&mut jacobians[*ground_jacobian_id..], mb.ndofs()).fill(N::zero());
            jacobians[*ground_jacobian_id + link.assembly_id() + dof_id] = N::one();

            let weighted_jacobian_id = *ground_jacobian_id + mb.ndofs();
            link.inv_mass_mul_unit_joint_force(
                dof_id,
                N::one(),
                &mut jacobians[weighted_jacobian_id..],
            );

            let inv_r = jacobians[weighted_jacobian_id + link.assembly_id() + dof_id]; // = J^t * M^-1 J

            let rhs;
            if err >= N::zero() {
                let stabilization = -err / params.dt * params.erp;
                rhs = dvel + stabilization;
            } else {
                rhs = -err / params.dt;
            }

            let constraint = UnilateralGroundConstraint {
                impulse: N::zero(),
                r: N::one() / inv_r,
                rhs: rhs,
                assembly_id: assembly_id,
                jacobian_id: *ground_jacobian_id,
                weighted_jacobian_id: *ground_jacobian_id + mb.ndofs(),
                ndofs: mb.ndofs(),
            };

            out_unilateral.push(constraint);
            *ground_jacobian_id += 2 * mb.ndofs();
        }
    }

    if let Some(max_position) = joint.max_position() {
        let err = -(max_position - joint.position());
        let dvel =
            -link.joint_velocity()[dof_id] - ext_vels[assembly_id + link.assembly_id() + dof_id];

        if err >= N::zero() || err <= dvel * params.dt {
            DVectorSliceMut::new(&mut jacobians[*ground_jacobian_id..], mb.ndofs()).fill(N::zero());
            jacobians[*ground_jacobian_id + link.assembly_id() + dof_id] = -N::one();
            let weighted_jacobian_id = *ground_jacobian_id + mb.ndofs();

            if is_min_constraint_active {
                // This jacobian is simply the negation of the first one.
                for i in 0..mb.ndofs() {
                    jacobians[weighted_jacobian_id + i] =
                        -jacobians[*ground_jacobian_id - mb.ndofs() + i];
                }
            } else {
                link.inv_mass_mul_unit_joint_force(
                    dof_id,
                    -N::one(),
                    &mut jacobians[weighted_jacobian_id..],
                );
            }

            let inv_r = -jacobians[weighted_jacobian_id + link.assembly_id() + dof_id]; // = J^t * M^-1 J

            let rhs;
            if err >= N::zero() {
                let stabilization = -err / params.dt * params.erp;
                rhs = dvel + stabilization;
            } else {
                rhs = -err / params.dt;
            }

            let constraint = UnilateralGroundConstraint {
                impulse: N::zero(),
                r: N::one() / inv_r,
                rhs: rhs,
                assembly_id: assembly_id,
                jacobian_id: *ground_jacobian_id,
                weighted_jacobian_id: *ground_jacobian_id + mb.ndofs(),
                ndofs: mb.ndofs(),
            };

            out_unilateral.push(constraint);
            *ground_jacobian_id += 2 * mb.ndofs();
        }
    }
}
