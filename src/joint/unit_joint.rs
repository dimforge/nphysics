#![allow(missing_docs)] // For downcast.

use na::{DVectorSliceMut, RealField};

use crate::joint::{Joint, JointMotor};
use crate::object::{BodyPartHandle, Multibody, MultibodyLink, Body};
use crate::solver::{BilateralGroundConstraint, ConstraintSet, GenericNonlinearConstraint,
             IntegrationParameters, UnilateralGroundConstraint};

/// Trait implemented by joints using the reduced-coordinates approach and allowing only one degree of freedom.
pub trait UnitJoint<N: RealField>: Joint<N> {
    /// The generalized coordinate of the unit joint.
    fn position(&self) -> N;
    /// The motor applied to the degree of freedom of the unit joitn.
    fn motor(&self) -> &JointMotor<N, N>;
    /// The lower limit, if any, set to the generalized coordinate of this unit joint.
    fn min_position(&self) -> Option<N>;
    /// The upper limit, if any, set to the generalized coordinate of this unit joint.
    fn max_position(&self) -> Option<N>;
}

impl_downcast!(UnitJoint<N> where N: RealField);

/// Computes the maximum number of velocity constraints to be applied by the given unit joint.
pub fn unit_joint_num_velocity_constraints<N: RealField, J: UnitJoint<N>>(joint: &J) -> usize {
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

/// Initializes and generate the velocity constraints applicable to the multibody links attached
/// to this joint.
pub fn unit_joint_velocity_constraints<N: RealField, J: UnitJoint<N>>(
    joint: &J,
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
    let ndofs = multibody.ndofs();
    let impulses = multibody.impulses();
    let mut is_min_constraint_active = false;
    let joint_velocity = multibody.joint_velocity(link);

    if joint.motor().enabled {
        let dvel = joint_velocity[dof_id] + ext_vels[link.assembly_id];

        DVectorSliceMut::from_slice(&mut jacobians[*ground_j_id..], ndofs).fill(N::zero());
        jacobians[*ground_j_id + link.assembly_id + dof_id] = N::one();

        let wj_id = *ground_j_id + ndofs;
        multibody.inv_mass_mul_unit_joint_force(link, dof_id, N::one(), &mut jacobians[wj_id..]);

        let inv_r = jacobians[wj_id + link.assembly_id + dof_id]; // = J^t * M^-1 J
        let rhs = dvel - joint.motor().desired_velocity;
        let limits = joint.motor().impulse_limits();
        let impulse_id = link.impulse_id + dof_id * 3;

        let constraint = BilateralGroundConstraint {
            impulse: impulses[impulse_id] * params.warmstart_coeff,
            r: N::one() / inv_r,
            rhs,
            limits,
            impulse_id,
            assembly_id,
            j_id: *ground_j_id,
            wj_id: *ground_j_id + ndofs,
            ndofs,
        };

        constraints.velocity.bilateral_ground.push(constraint);
        *ground_j_id += 2 * ndofs;
    }

    if let Some(min_position) = joint.min_position() {
        let err = min_position - joint.position();
        let dvel =
            joint_velocity[dof_id] + ext_vels[link.assembly_id + dof_id];

        if err >= N::zero() {
            is_min_constraint_active = true;
            DVectorSliceMut::from_slice(&mut jacobians[*ground_j_id..], ndofs).fill(N::zero());
            jacobians[*ground_j_id + link.assembly_id + dof_id] = N::one();

            let wj_id = *ground_j_id + ndofs;
            multibody.inv_mass_mul_unit_joint_force(link, dof_id, N::one(), &mut jacobians[wj_id..]);

            let inv_r = jacobians[wj_id + link.assembly_id + dof_id]; // = J^t * M^-1 J

            let impulse_id = link.impulse_id + dof_id * 3 + 1;
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
            -joint_velocity[dof_id] - ext_vels[link.assembly_id + dof_id];

        if err >= N::zero() {
            DVectorSliceMut::from_slice(&mut jacobians[*ground_j_id..], ndofs).fill(N::zero());
            jacobians[*ground_j_id + link.assembly_id + dof_id] = -N::one();
            let wj_id = *ground_j_id + ndofs;

            if is_min_constraint_active {
                // This jacobian is simply the negation of the first one.
                for i in 0..ndofs {
                    jacobians[wj_id + i] = -jacobians[*ground_j_id - ndofs + i];
                }
            } else {
                multibody.inv_mass_mul_unit_joint_force(link, dof_id, -N::one(), &mut jacobians[wj_id..]);
            }

            let inv_r = -jacobians[wj_id + link.assembly_id + dof_id]; // = J^t * M^-1 J

            let impulse_id = link.impulse_id + dof_id * 3 + 2;
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
}

/// Initializes and generate the position constraints applicable to the multibody links attached
/// to this joint.
pub fn unit_joint_position_constraint<N: RealField, J: UnitJoint<N>>(
    joint: &J,
    multibody: &Multibody<N>,
    link: &MultibodyLink<N>,
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
        let ndofs = multibody.ndofs();

        multibody.inv_mass_mul_unit_joint_force(link, dof_id, sign, jacobians);

        let inv_r = sign * jacobians[link.assembly_id + dof_id]; // = J^t * M^-1 J

        return Some(GenericNonlinearConstraint::new(
            link.part_handle(),
            BodyPartHandle::ground(),
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
