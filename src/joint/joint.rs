use downcast::Any;

use na::{DVectorSliceMut, Real};

use joint::JointMotor;
use solver::{BilateralGroundConstraint, ConstraintSet, GenericNonlinearConstraint,
             IntegrationParameters, UnilateralGroundConstraint};
use object::{BodyHandle, Multibody, MultibodyLinkRef};
use math::{Isometry, JacobianSliceMut, Vector, Velocity};

pub trait Joint<N: Real>: Any + Send + Sync {
    fn ndofs(&self) -> usize;
    fn body_to_parent(&self, parent_shift: &Vector<N>, body_shift: &Vector<N>) -> Isometry<N>;
    fn update_jacobians(&mut self, body_shift: &Vector<N>, vels: &[N]);
    fn integrate(&mut self, params: &IntegrationParameters<N>, vels: &[N]);
    fn apply_displacement(&mut self, disp: &[N]);

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

    /// The maximum number of impulses needed by this joints for
    /// its constraints.
    fn nimpulses(&self) -> usize {
        // FIXME: keep this?
        self.ndofs() * 3
    }

    fn nconstraints(&self) -> usize {
        0
    }

    fn build_constraints(
        &self,
        _params: &IntegrationParameters<N>,
        _link: &MultibodyLinkRef<N>,
        _assembly_id: usize,
        _dof_id: usize,
        _ext_vels: &[N],
        _ground_jacobian_id: &mut usize,
        _jacobians: &mut [N],
        _velocity_constraints: &mut ConstraintSet<N>,
    ) {
    }

    fn nposition_constraints(&self) -> usize {
        0
    }

    fn position_constraint(
        &self,
        _i: usize,
        _link: &MultibodyLinkRef<N>,
        _dof_id: usize,
        _jacobians: &mut [N],
    ) -> Option<GenericNonlinearConstraint<N>> {
        None
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
    link: &MultibodyLinkRef<N>,
    assembly_id: usize,
    dof_id: usize,
    ext_vels: &[N],
    ground_jacobian_id: &mut usize,
    jacobians: &mut [N],
    constraints: &mut ConstraintSet<N>,
) {
    let ndofs = link.multibody().ndofs();
    let impulses = link.multibody().impulses();
    let mut is_min_constraint_active = false;

    if joint.motor().enabled {
        let dvel = link.joint_velocity()[dof_id] + ext_vels[assembly_id + link.assembly_id()];

        DVectorSliceMut::new(&mut jacobians[*ground_jacobian_id..], ndofs).fill(N::zero());
        jacobians[*ground_jacobian_id + link.assembly_id() + dof_id] = N::one();

        let weighted_jacobian_id = *ground_jacobian_id + ndofs;
        link.inv_mass_mul_unit_joint_force(
            dof_id,
            N::one(),
            &mut jacobians[weighted_jacobian_id..],
        );

        let inv_r = jacobians[weighted_jacobian_id + link.assembly_id() + dof_id]; // = J^t * M^-1 J
        let rhs = dvel - joint.motor().desired_velocity;
        let limits = joint.motor().impulse_limits();
        let cache_id = link.impulse_id() + dof_id * 3;

        let constraint = BilateralGroundConstraint {
            impulse: impulses[cache_id] * params.warmstart_coeff,
            r: N::one() / inv_r,
            rhs: rhs,
            limits: limits,
            cache_id: cache_id,
            assembly_id: assembly_id,
            jacobian_id: *ground_jacobian_id,
            weighted_jacobian_id: *ground_jacobian_id + ndofs,
            ndofs: ndofs,
        };

        constraints.velocity.bilateral_ground.push(constraint);
        *ground_jacobian_id += 2 * ndofs;
    }

    if let Some(min_position) = joint.min_position() {
        let err = min_position - joint.position();
        let dvel =
            link.joint_velocity()[dof_id] + ext_vels[assembly_id + link.assembly_id() + dof_id];

        if err >= N::zero() || err <= dvel * params.dt {
            is_min_constraint_active = true;
            DVectorSliceMut::new(&mut jacobians[*ground_jacobian_id..], ndofs).fill(N::zero());
            jacobians[*ground_jacobian_id + link.assembly_id() + dof_id] = N::one();

            let weighted_jacobian_id = *ground_jacobian_id + ndofs;
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

            let cache_id = link.impulse_id() + dof_id * 3 + 1;
            let constraint = UnilateralGroundConstraint {
                impulse: impulses[cache_id] * params.warmstart_coeff,
                r: N::one() / inv_r,
                rhs,
                cache_id,
                assembly_id,
                jacobian_id: *ground_jacobian_id,
                weighted_jacobian_id: *ground_jacobian_id + ndofs,
                ndofs,
            };

            constraints.velocity.unilateral_ground.push(constraint);
            *ground_jacobian_id += 2 * ndofs;
        }
    }

    if let Some(max_position) = joint.max_position() {
        let err = -(max_position - joint.position());
        let dvel =
            -link.joint_velocity()[dof_id] - ext_vels[assembly_id + link.assembly_id() + dof_id];

        if err >= N::zero() || err <= dvel * params.dt {
            DVectorSliceMut::new(&mut jacobians[*ground_jacobian_id..], ndofs).fill(N::zero());
            jacobians[*ground_jacobian_id + link.assembly_id() + dof_id] = -N::one();
            let weighted_jacobian_id = *ground_jacobian_id + ndofs;

            if is_min_constraint_active {
                // This jacobian is simply the negation of the first one.
                for i in 0..ndofs {
                    jacobians[weighted_jacobian_id + i] =
                        -jacobians[*ground_jacobian_id - ndofs + i];
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

            let cache_id = link.impulse_id() + dof_id * 3 + 2;
            let constraint = UnilateralGroundConstraint {
                impulse: impulses[cache_id] * params.warmstart_coeff,
                r: N::one() / inv_r,
                rhs: rhs,
                cache_id: cache_id,
                assembly_id: assembly_id,
                jacobian_id: *ground_jacobian_id,
                weighted_jacobian_id: *ground_jacobian_id + ndofs,
                ndofs: ndofs,
            };

            constraints.velocity.unilateral_ground.push(constraint);
            *ground_jacobian_id += 2 * ndofs;
        }
    }
}

pub fn unit_joint_position_constraint<N: Real, J: UnitJoint<N>>(
    joint: &J,
    link: &MultibodyLinkRef<N>,
    dof_id: usize,
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
            ndofs,
            0,
            rhs,
            inv_r,
        ));
    }

    None
}
