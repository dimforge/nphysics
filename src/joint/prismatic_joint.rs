#![macro_use]

use na::{self, DVectorSliceMut, RealField, Unit};

use crate::joint::{self, Joint, JointMotor, UnitJoint};
use crate::math::{Dim, Isometry, JacobianSliceMut, Rotation, Translation, Vector, Velocity};
use crate::object::{MultibodyLink, Multibody};
use crate::solver::{ConstraintSet, GenericNonlinearConstraint, IntegrationParameters};

/// A unit joint that allows only one translational degree on freedom.
#[derive(Copy, Clone, Debug)]
pub struct PrismaticJoint<N: RealField> {
    axis: Unit<Vector<N>>,
    jacobian: Velocity<N>,

    offset: N,

    min_offset: Option<N>,
    max_offset: Option<N>,
    motor: JointMotor<N, N>,
}

impl<N: RealField> PrismaticJoint<N> {
    /// Create a new prismatic joint where the allowed traslation is defined along the provided axis.
    ///
    /// The axis is expressed in the local coordinate system of the two multibody links attached to this joint.
    #[cfg(feature = "dim2")]
    pub fn new(axis: Unit<Vector<N>>, offset: N) -> Self {
        PrismaticJoint {
            axis: axis,
            jacobian: Velocity::zero(),
            offset: offset,
            min_offset: None,
            max_offset: None,
            motor: JointMotor::new(),
        }
    }

    /// Create a new prismatic joint where the allowed traslation is defined along the provided axis.
    ///
    /// The axis is expressed in the local coordinate system of the two multibody links attached to this joint.
    #[cfg(feature = "dim3")]
    pub fn new(axis: Unit<Vector<N>>, offset: N) -> Self {
        PrismaticJoint {
            axis: axis,
            jacobian: Velocity::zero(),
            offset: offset,
            min_offset: None,
            max_offset: None,
            motor: JointMotor::new(),
        }
    }

    /// The relative displacement of the attached multibody links along the joint axis.
    pub fn offset(&self) -> N {
        self.offset
    }

    /// The relative translation of the attached multibody links along the joint axis.
    pub fn translation(&self) -> Translation<N> {
        Translation::from(*self.axis * self.offset)
    }

    /// The lower limit of the relative displacement of the attached multibody links along the joint axis.
    pub fn min_offset(&self) -> Option<N> {
        self.min_offset
    }

    /// The upper limit of the relative displacement of the attached multibody links along the joint axis.
    pub fn max_offset(&self) -> Option<N> {
        self.max_offset
    }

    /// Disable the lower limit of the relative displacement of the attached multibody links along the joint axis.
    pub fn disable_min_offset(&mut self) {
        self.min_offset = None;
    }

    /// Disable the upper limit of the relative displacement of the attached multibody links along the joint axis.
    pub fn disable_max_offset(&mut self) {
        self.max_offset = None;
    }

    /// Set the lower limit of the relative displacement of the attached multibody links along the joint axis.
    pub fn enable_min_offset(&mut self, limit: N) {
        self.min_offset = Some(limit);
        self.assert_limits();
    }

    /// Set the upper limit of the relative displacement of the attached multibody links along the joint axis.
    pub fn enable_max_offset(&mut self, limit: N) {
        self.max_offset = Some(limit);
        self.assert_limits();
    }

    /// Returns `true` if the joint motor is enabled.
    pub fn is_linear_motor_enabled(&self) -> bool {
        self.motor.enabled
    }

    /// Enable the joint motor.
    pub fn enable_linear_motor(&mut self) {
        self.motor.enabled = true
    }

    /// Disable the joint motor.
    pub fn disable_linear_motor(&mut self) {
        self.motor.enabled = false;
    }

    /// The desired relative velocity to be enforced by the joint motor.
    pub fn desired_linear_motor_velocity(&self) -> N {
        self.motor.desired_velocity
    }

    /// Set the desired relative velocity to be enforced by the joint motor.
    pub fn set_desired_linear_motor_velocity(&mut self, vel: N) {
        self.motor.desired_velocity = vel;
    }

    /// The maximum force that can be output by the joint motor.
    pub fn max_linear_motor_force(&self) -> N {
        self.motor.max_force
    }

    /// Set the maximum force that can be output by the joint motor.
    pub fn set_max_linear_motor_force(&mut self, force: N) {
        self.motor.max_force = force;
    }

    fn assert_limits(&self) {
        if let (Some(min_offset), Some(max_offset)) = (self.min_offset, self.max_offset) {
            assert!(
                min_offset <= max_offset,
                "PrismaticJoint joint limits: the min offset must be larger than (or equal to) the max offset.");
        }
    }
}

impl<N: RealField> Joint<N> for PrismaticJoint<N> {
    #[inline]
    fn clone(&self) -> Box<Joint<N>> {
        Box::new(*self)
    }

    #[inline]
    fn ndofs(&self) -> usize {
        1
    }

    #[cfg(feature = "dim3")]
    fn body_to_parent(&self, parent_shift: &Vector<N>, body_shift: &Vector<N>) -> Isometry<N> {
        let trans =
            Translation::from(parent_shift - body_shift + self.axis.as_ref() * self.offset);
        Isometry::from_parts(trans, Rotation::identity())
    }

    #[cfg(feature = "dim2")]
    fn body_to_parent(&self, parent_shift: &Vector<N>, body_shift: &Vector<N>) -> Isometry<N> {
        let trans =
            Translation::from(parent_shift - body_shift + self.axis.as_ref() * self.offset);
        Isometry::from_parts(trans, Rotation::identity())
    }

    fn update_jacobians(&mut self, _: &Vector<N>, _: &[N]) {}

    fn jacobian(&self, transform: &Isometry<N>, out: &mut JacobianSliceMut<N>) {
        let transformed_axis = transform * self.axis;
        out.fixed_rows_mut::<Dim>(0)
            .copy_from(transformed_axis.as_ref())
    }

    fn jacobian_dot(&self, _: &Isometry<N>, _: &mut JacobianSliceMut<N>) {}

    fn jacobian_dot_veldiff_mul_coordinates(
        &self,
        _: &Isometry<N>,
        _: &[N],
        _: &mut JacobianSliceMut<N>,
    ) {}

    fn default_damping(&self, _: &mut DVectorSliceMut<N>) {}

    fn integrate(&mut self, params: &IntegrationParameters<N>, vels: &[N]) {
        self.offset += vels[0] * params.dt
    }

    fn apply_displacement(&mut self, disp: &[N]) {
        self.offset += disp[0]
    }

    fn jacobian_mul_coordinates(&self, acc: &[N]) -> Velocity<N> {
        Velocity::new(self.axis.as_ref() * acc[0], na::zero())
    }

    fn jacobian_dot_mul_coordinates(&self, _: &[N]) -> Velocity<N> {
        Velocity::zero()
    }

    fn num_velocity_constraints(&self) -> usize {
        joint::unit_joint_num_velocity_constraints(self)
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
        joint::unit_joint_velocity_constraints(
            self,
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
    }

    fn num_position_constraints(&self) -> usize {
        if self.min_offset.is_some() || self.max_offset.is_some() {
            1
        } else {
            0
        }
    }

    fn position_constraint(
        &self,
        _: usize,
        multibody: &Multibody<N>,
        link: &MultibodyLink<N>,
        dof_id: usize,
        jacobians: &mut [N],
    ) -> Option<GenericNonlinearConstraint<N>> {
        joint::unit_joint_position_constraint(self, multibody, link, dof_id, false, jacobians)
    }
}

impl<N: RealField> UnitJoint<N> for PrismaticJoint<N> {
    fn position(&self) -> N {
        self.offset
    }

    fn motor(&self) -> &JointMotor<N, N> {
        &self.motor
    }

    fn min_position(&self) -> Option<N> {
        self.min_offset
    }

    fn max_position(&self) -> Option<N> {
        self.max_offset
    }
}

#[cfg(feature = "dim3")]
macro_rules! prismatic_motor_limit_methods (
    ($ty: ident, $prism: ident) => {
        _prismatic_motor_limit_methods!(
            $ty,
            $prism,
            min_offset,
            max_offset,
            disable_min_offset,
            disable_max_offset,
            enable_min_offset,
            enable_max_offset,
            is_linear_motor_enabled,
            enable_linear_motor,
            disable_linear_motor,
            desired_linear_motor_velocity,
            set_desired_linear_motor_velocity,
            max_linear_motor_force,
            set_max_linear_motor_force);
    }
);

#[cfg(feature = "dim3")]
macro_rules! prismatic_motor_limit_methods_1 (
    ($ty: ident, $prism: ident) => {
        _prismatic_motor_limit_methods!(
            $ty,
            $prism,
            min_offset_1,
            max_offset_1,
            disable_min_offset_1,
            disable_max_offset_1,
            enable_min_offset_1,
            enable_max_offset_1,
            is_linear_motor_enabled_1,
            enable_linear_motor_1,
            disable_linear_motor_1,
            desired_linear_motor_velocity_1,
            set_desired_linear_motor_velocity_1,
            max_linear_motor_force_1,
            set_max_linear_motor_force_1);
    }
);

#[cfg(feature = "dim3")]
macro_rules! prismatic_motor_limit_methods_2 (
    ($ty: ident, $prism: ident) => {
        _prismatic_motor_limit_methods!(
            $ty,
            $prism,
            min_offset_2,
            max_offset_2,
            disable_min_offset_2,
            disable_max_offset_2,
            enable_min_offset_2,
            enable_max_offset_2,
            is_linear_motor_enabled_2,
            enable_linear_motor_2,
            disable_linear_motor_2,
            desired_linear_motor_velocity_2,
            set_desired_linear_motor_velocity_2,
            max_linear_motor_force2,
            set_max_linear_motor_force_2);
    }
);

#[cfg(feature = "dim3")]
macro_rules! _prismatic_motor_limit_methods (
    ($ty: ident, $prism: ident,
     $min_offset:         ident,
     $max_offset:         ident,
     $disable_min_offset: ident,
     $disable_max_offset: ident,
     $enable_min_offset:  ident,
     $enable_max_offset:  ident,
     $is_motor_enabled:  ident,
     $enable_motor:      ident,
     $disable_motor:     ident,
     $desired_motor_velocity:     ident,
     $set_desired_motor_velocity: ident,
     $max_motor_force:           ident,
     $set_max_motor_force:       ident
     ) => {
        impl<N: RealField> $ty<N> {
            /// The lower limit of the relative translational displacement of the attached multibody links along the joint axis.
            pub fn $min_offset(&self) -> Option<N> {
                self.$prism.min_offset()
            }

            /// The upper limit of the relative translational displacement of the attached multibody links along the joint axis.
            pub fn $max_offset(&self) -> Option<N> {
                self.$prism.max_offset()
            }

            /// Disable the lower limit of the relative translational displacement of the attached multibody links along the joint axis.
            pub fn $disable_min_offset(&mut self) {
                self.$prism.disable_max_offset();
            }

            /// Disable the upper limit of the relative translational displacement of the attached multibody links along the joint axis.
            pub fn $disable_max_offset(&mut self) {
                self.$prism.disable_max_offset();
            }

            /// Set the lower limit of the relative translational displacement of the attached multibody links along the joint axis.
            pub fn $enable_min_offset(&mut self, limit: N) {
                self.$prism.enable_min_offset(limit);
            }

            /// Set the upper limit of the relative translational displacement of the attached multibody links along the joint axis.
            pub fn $enable_max_offset(&mut self, limit: N) {
                self.$prism.enable_max_offset(limit)
            }

            /// Returns `true` if the joint translational motor is enabled.
            pub fn $is_motor_enabled(&self) -> bool {
                self.$prism.is_linear_motor_enabled()
            }

            /// Enable the joint translational motor.
            pub fn $enable_motor(&mut self) {
                self.$prism.enable_linear_motor()
            }

            /// Disable the joint translational motor.
            pub fn $disable_motor(&mut self) {
                self.$prism.disable_linear_motor()
            }

            /// The desired relative translational velocity to be enforced by the joint motor.
            pub fn $desired_motor_velocity(&self) -> N {
                self.$prism.desired_linear_motor_velocity()
            }

            /// Set the desired relative translational velocity to be enforced by the joint motor.
            pub fn $set_desired_motor_velocity(&mut self, vel: N) {
                self.$prism.set_desired_linear_motor_velocity(vel)
            }

            /// The maximum force that can be output by the joint translational motor.
            pub fn $max_motor_force(&self) -> N {
                self.$prism.max_linear_motor_force()
            }

            /// Set the maximum force that can be output by the joint translational motor.
            pub fn $set_max_motor_force(&mut self, force: N) {
                self.$prism.set_max_linear_motor_force(force)
            }
        }
    }
);
