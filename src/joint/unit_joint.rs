use downcast::Any;

use na::{DVectorSliceMut, Real};

use joint::{Joint, JointMotor};
use math::{Isometry, JacobianSliceMut, Vector, Velocity};
use object::{BodyHandle, Multibody, MultibodyLinkRef};
use solver::{BilateralGroundConstraint, ConstraintSet, GenericNonlinearConstraint,
             IntegrationParameters, UnilateralGroundConstraint};

// FIXME: move this to its own file.
pub trait UnitJoint<N: Real>: Joint<N> {
    fn position(&self) -> N;
    fn motor(&self) -> &JointMotor<N, N>;
    fn min_position(&self) -> Option<N>;
    fn max_position(&self) -> Option<N>;
}

downcast!(<N> UnitJoint<N> where N: Real);

pub fn unit_joint_num_velocity_constraints<N: Real, J: UnitJoint<N>>(joint: &J) -> usize {
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

pub fn unit_joint_velocity_constraints<N: Real, J: UnitJoint<N>>(
    joint: &J,
    params: &IntegrationParameters<N>,
    link: &MultibodyLinkRef<N>,
    assembly_id: usize,
    dof_id: usize,
    ext_vels: &[N],
    ground_j_id: &mut usize,
    jacobians: &mut [N],
    constraints: &mut ConstraintSet<N>,
) {
    let ndofs = link.multibody().ndofs();
    let impulses = link.multibody().impulses();
    let mut is_min_constraint_active = false;

    if joint.motor().enabled {
        let dvel = link.joint_velocity()[dof_id] + ext_vels[assembly_id + link.assembly_id()];

        DVectorSliceMut::from_slice(&mut jacobians[*ground_j_id..], ndofs).fill(N::zero());
        jacobians[*ground_j_id + link.assembly_id() + dof_id] = N::one();

        let wj_id = *ground_j_id + ndofs;
        link.inv_mass_mul_unit_joint_force(dof_id, N::one(), &mut jacobians[wj_id..]);

        let inv_r = jacobians[wj_id + link.assembly_id() + dof_id]; // = J^t * M^-1 J
        let rhs = dvel - joint.motor().desired_velocity;
        let limits = joint.motor().impulse_limits();
        let impulse_id = link.impulse_id() + dof_id * 3;

        let constraint = BilateralGroundConstraint {
            impulse: impulses[impulse_id] * params.warmstart_coeff,
            r: N::one() / inv_r,
            rhs: rhs,
            limits: limits,
            impulse_id: impulse_id,
            assembly_id: assembly_id,
            j_id: *ground_j_id,
            wj_id: *ground_j_id + ndofs,
            ndofs: ndofs,
        };

        constraints.velocity.bilateral_ground.push(constraint);
        *ground_j_id += 2 * ndofs;
    }

    if let Some(min_position) = joint.min_position() {
        let err = min_position - joint.position();
        let dvel =
            link.joint_velocity()[dof_id] + ext_vels[assembly_id + link.assembly_id() + dof_id];

        if err >= N::zero() {
            is_min_constraint_active = true;
            DVectorSliceMut::from_slice(&mut jacobians[*ground_j_id..], ndofs).fill(N::zero());
            jacobians[*ground_j_id + link.assembly_id() + dof_id] = N::one();

            let wj_id = *ground_j_id + ndofs;
            link.inv_mass_mul_unit_joint_force(dof_id, N::one(), &mut jacobians[wj_id..]);

            let inv_r = jacobians[wj_id + link.assembly_id() + dof_id]; // = J^t * M^-1 J

            let impulse_id = link.impulse_id() + dof_id * 3 + 1;
            let constraint = UnilateralGroundConstraint {
                impulse: impulses[impulse_id] * params.warmstart_coeff,
                r: N::one() / inv_r,
                rhs: dvel,
                impulse_id,
                assembly_id,
                j_id: *ground_j_id,
                wj_id: *ground_j_id + ndofs,
                ndofs,
            };

            constraints.velocity.unilateral_ground.push(constraint);
            *ground_j_id += 2 * ndofs;
        }
    }

    if let Some(max_position) = joint.max_position() {
        let err = -(max_position - joint.position());
        let dvel =
            -link.joint_velocity()[dof_id] - ext_vels[assembly_id + link.assembly_id() + dof_id];

        if err >= N::zero() {
            DVectorSliceMut::from_slice(&mut jacobians[*ground_j_id..], ndofs).fill(N::zero());
            jacobians[*ground_j_id + link.assembly_id() + dof_id] = -N::one();
            let wj_id = *ground_j_id + ndofs;

            if is_min_constraint_active {
                // This jacobian is simply the negation of the first one.
                for i in 0..ndofs {
                    jacobians[wj_id + i] = -jacobians[*ground_j_id - ndofs + i];
                }
            } else {
                link.inv_mass_mul_unit_joint_force(dof_id, -N::one(), &mut jacobians[wj_id..]);
            }

            let inv_r = -jacobians[wj_id + link.assembly_id() + dof_id]; // = J^t * M^-1 J

            let impulse_id = link.impulse_id() + dof_id * 3 + 2;
            let constraint = UnilateralGroundConstraint {
                impulse: impulses[impulse_id] * params.warmstart_coeff,
                r: N::one() / inv_r,
                rhs: dvel,
                impulse_id: impulse_id,
                assembly_id: assembly_id,
                j_id: *ground_j_id,
                wj_id: *ground_j_id + ndofs,
                ndofs: ndofs,
            };

            constraints.velocity.unilateral_ground.push(constraint);
            *ground_j_id += 2 * ndofs;
        }
    }
}

pub fn unit_joint_position_constraint<N: Real, J: UnitJoint<N>>(
    joint: &J,
    link: &MultibodyLinkRef<N>,
    dof_id: usize,
    is_angular: bool,
    jacobians: &mut [N],
) -> Option<GenericNonlinearConstraint<N>> {
    let mut sign = N::one();
    let mut rhs = None;

    if let Some(min_position) = joint.min_position() {
        let err = min_position - joint.position();
        if err > N::zero() {
            rhs = Some(-err);
        }
    }

    if rhs.is_none() {
        if let Some(max_position) = joint.max_position() {
            let err = -(max_position - joint.position());
            if err > N::zero() {
                rhs = Some(-err);
                sign = -N::one();
            }
        }
    }

    if let Some(rhs) = rhs {
        let mb = link.multibody();
        let ndofs = mb.ndofs();

        link.inv_mass_mul_unit_joint_force(dof_id, sign, jacobians);

        let inv_r = sign * jacobians[link.assembly_id() + dof_id]; // = J^t * M^-1 J

        return Some(GenericNonlinearConstraint::new(
            mb.handle(),
            BodyHandle::ground(),
            is_angular,
            ndofs,
            0,
            0,
            0,
            rhs,
            N::one() / inv_r,
        ));
    }

    None
}
