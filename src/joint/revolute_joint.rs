#![macro_use]

use na::{self, DVectorSliceMut, RealField, Unit};

use crate::joint::{self, Joint, JointMotor, UnitJoint};
use crate::math::{AngularVector, Isometry, JacobianSliceMut, Rotation, Translation, Vector, Velocity};
use crate::object::{MultibodyLink, Multibody};
use crate::solver::{ConstraintSet, GenericNonlinearConstraint, IntegrationParameters};
use crate::utils::GeneralizedCross;

/// A unit joint that allows only one relative rotational degree of freedom between two multibody links.
#[derive(Copy, Clone, Debug)]
pub struct RevoluteJoint<N: RealField> {
    axis: Unit<AngularVector<N>>,
    jacobian: Velocity<N>,
    jacobian_dot: Velocity<N>,
    jacobian_dot_veldiff: Velocity<N>,
    rot: Rotation<N>,

    angle: N,

    min_angle: Option<N>,
    max_angle: Option<N>,
    motor: JointMotor<N, N>,
}

impl<N: RealField> RevoluteJoint<N> {
    /// Create a new revolute joint with an initial angle.
    #[cfg(feature = "dim2")]
    pub fn new(angle: N) -> Self {
        RevoluteJoint {
            axis: AngularVector::x_axis(),
            jacobian: Velocity::zero(),
            jacobian_dot: Velocity::zero(),
            jacobian_dot_veldiff: Velocity::zero(),
            rot: Rotation::new(angle),
            angle: angle,
            min_angle: None,
            max_angle: None,
            motor: JointMotor::new(),
        }
    }

    /// Create a new revolute joint with an axis and an initial angle.
    ///
    /// The axis along which the rotation can happen is expressed in the local coordinate
    /// system of the attached multibody links.
    #[cfg(feature = "dim3")]
    pub fn new(axis: Unit<AngularVector<N>>, angle: N) -> Self {
        RevoluteJoint {
            axis: axis,
            jacobian: Velocity::zero(),
            jacobian_dot: Velocity::zero(),
            jacobian_dot_veldiff: Velocity::zero(),
            rot: Rotation::from_axis_angle(&axis, angle),
            angle: angle,
            min_angle: None,
            max_angle: None,
            motor: JointMotor::new(),
        }
    }

    #[cfg(feature = "dim3")]
    fn update_rot(&mut self) {
        self.rot = Rotation::from_axis_angle(&self.axis, self.angle);
    }

    #[cfg(feature = "dim2")]
    fn update_rot(&mut self) {
        self.rot = Rotation::from_angle(self.angle);
    }

    /// The axis of the rotational degree of freedom.
    pub fn axis(&self) -> Unit<AngularVector<N>> {
        self.axis
    }

    /// The angle of rotation.
    pub fn angle(&self) -> N {
        self.angle
    }

    /// The rotation from an attached multibody link to its dependent.
    pub fn rotation(&self) -> &Rotation<N> {
        &self.rot
    }

    /// The jacobian of this joint expressed in the local coordinate frame of the joint.
    pub fn local_jacobian(&self) -> &Velocity<N> {
        &self.jacobian
    }

    /// The time-derivative of the jacobian of this joint expressed in the local coordinate frame of the joint.
    pub fn local_jacobian_dot(&self) -> &Velocity<N> {
        &self.jacobian_dot
    }

    /// The velocity-derivative of the time-derivative of the jacobian of this joint expressed in the local coordinate frame of the joint.
    pub fn local_jacobian_dot_veldiff(&self) -> &Velocity<N> {
        &self.jacobian_dot_veldiff
    }

    /// The lower limit of the rotation angle.
    pub fn min_angle(&self) -> Option<N> {
        self.min_angle
    }

    /// The upper limit of the rotation angle.
    pub fn max_angle(&self) -> Option<N> {
        self.max_angle
    }

    /// Disable the lower limit of the rotation angle.
    pub fn disable_min_angle(&mut self) {
        self.min_angle = None;
    }

    /// Disable the upper limit of the rotation angle.
    pub fn disable_max_angle(&mut self) {
        self.max_angle = None;
    }

    /// Enable and set the lower limit of the rotation angle.
    pub fn enable_min_angle(&mut self, limit: N) {
        self.min_angle = Some(limit);
        self.assert_limits();
    }

    /// Enable and set the upper limit of the rotation angle.
    pub fn enable_max_angle(&mut self, limit: N) {
        self.max_angle = Some(limit);
        self.assert_limits();
    }

    /// Return `true` if the angular motor of this joint is enabled.
    pub fn is_angular_motor_enabled(&self) -> bool {
        self.motor.enabled
    }

    /// Enable the angular motor of this joint.
    pub fn enable_angular_motor(&mut self) {
        self.motor.enabled = true
    }

    /// Disable the angular motor of this joint.
    pub fn disable_angular_motor(&mut self) {
        self.motor.enabled = false;
    }

    /// The desired angular velocity of the joint motor.
    pub fn desired_angular_motor_velocity(&self) -> N {
        self.motor.desired_velocity
    }

    /// Set the desired angular velocity of the joint motor.
    pub fn set_desired_angular_motor_velocity(&mut self, vel: N) {
        self.motor.desired_velocity = vel;
    }

    /// The maximum torque that can be delivered by the joint motor.
    pub fn max_angular_motor_torque(&self) -> N {
        self.motor.max_force
    }

    /// Set the maximum torque that can be delivered by the joint motor.
    pub fn set_max_angular_motor_torque(&mut self, torque: N) {
        self.motor.max_force = torque;
    }

    fn assert_limits(&self) {
        if let (Some(min_angle), Some(max_angle)) = (self.min_angle, self.max_angle) {
            assert!(
                min_angle <= max_angle,
                "RevoluteJoint joint limits: the min angle must be larger than (or equal to) the max angle.");
        }
    }
}

impl<N: RealField> Joint<N> for RevoluteJoint<N> {
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
        let trans = Translation::from(parent_shift - self.rot * body_shift);
        Isometry::from_parts(trans, self.rot)
    }

    #[cfg(feature = "dim2")]
    fn body_to_parent(&self, parent_shift: &Vector<N>, body_shift: &Vector<N>) -> Isometry<N> {
        let trans = Translation::from(parent_shift - self.rot * body_shift);
        Isometry::from_parts(trans, self.rot)
    }

    fn update_jacobians(&mut self, body_shift: &Vector<N>, vels: &[N]) {
        let shift = self.rot * -body_shift;
        let shift_dot_veldiff = self.axis.gcross(&shift);

        self.jacobian = Velocity::new_with_vectors(self.axis.gcross(&shift), self.axis.into_inner());
        self.jacobian_dot_veldiff.linear = self.axis.gcross(&shift_dot_veldiff);
        self.jacobian_dot.linear = self.jacobian_dot_veldiff.linear * vels[0];
    }

    fn jacobian(&self, transform: &Isometry<N>, out: &mut JacobianSliceMut<N>) {
        out.copy_from(self.jacobian.transformed(transform).as_vector())
    }

    fn jacobian_dot(&self, transform: &Isometry<N>, out: &mut JacobianSliceMut<N>) {
        out.copy_from(self.jacobian_dot.transformed(transform).as_vector())
    }

    fn jacobian_dot_veldiff_mul_coordinates(
        &self,
        transform: &Isometry<N>,
        acc: &[N],
        out: &mut JacobianSliceMut<N>,
    ) {
        out.copy_from((self.jacobian_dot_veldiff.transformed(transform) * acc[0]).as_vector())
    }

    fn integrate(&mut self, params: &IntegrationParameters<N>, vels: &[N]) {
        self.angle += vels[0] * params.dt;
        self.update_rot();
    }

    fn default_damping(&self, out: &mut DVectorSliceMut<N>) {
        out.fill(na::convert(0.1f64))
    }

    fn apply_displacement(&mut self, disp: &[N]) {
        self.angle += disp[0];
        self.update_rot();
    }

    fn jacobian_mul_coordinates(&self, acc: &[N]) -> Velocity<N> {
        self.jacobian * acc[0]
    }

    fn jacobian_dot_mul_coordinates(&self, acc: &[N]) -> Velocity<N> {
        self.jacobian_dot * acc[0]
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
        )
    }

    fn num_position_constraints(&self) -> usize {
        if self.min_angle.is_some() || self.max_angle.is_some() {
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
        joint::unit_joint_position_constraint(self, multibody, link, dof_id, true, jacobians)
    }
}

impl<N: RealField> UnitJoint<N> for RevoluteJoint<N> {
    fn position(&self) -> N {
        self.angle
    }

    fn motor(&self) -> &JointMotor<N, N> {
        &self.motor
    }

    fn min_position(&self) -> Option<N> {
        self.min_angle
    }

    fn max_position(&self) -> Option<N> {
        self.max_angle
    }
}

#[cfg(feature = "dim3")]
macro_rules! revolute_motor_limit_methods (
    ($ty: ident, $revo: ident) => {
        _revolute_motor_limit_methods!(
            $ty,
            $revo,
            min_angle,
            max_angle,
            disable_min_angle,
            disable_max_angle,
            enable_min_angle,
            enable_max_angle,
            is_angular_motor_enabled,
            enable_angular_motor,
            disable_angular_motor,
            desired_angular_motor_velocity,
            set_desired_angular_motor_velocity,
            max_angular_motor_torque,
            set_max_angular_motor_torque);
    }
);

#[cfg(feature = "dim3")]
macro_rules! revolute_motor_limit_methods_1 (
    ($ty: ident, $revo: ident) => {
        _revolute_motor_limit_methods!(
            $ty,
            $revo,
            min_angle_1,
            max_angle_1,
            disable_min_angle_1,
            disable_max_angle_1,
            enable_min_angle_1,
            enable_max_angle_1,
            is_angular_motor_enabled_1,
            enable_angular_motor_1,
            disable_angular_motor_1,
            desired_angular_motor_velocity_1,
            set_desired_angular_motor_velocity_1,
            max_angular_motor_torque_1,
            set_max_angular_motor_torque_1);
    }
);

#[cfg(feature = "dim3")]
macro_rules! revolute_motor_limit_methods_2 (
    ($ty: ident, $revo: ident) => {
        _revolute_motor_limit_methods!(
            $ty,
            $revo,
            min_angle_2,
            max_angle_2,
            disable_min_angle_2,
            disable_max_angle_2,
            enable_min_angle_2,
            enable_max_angle_2,
            is_angular_motor_enabled_2,
            enable_angular_motor_2,
            disable_angular_motor_2,
            desired_angular_motor_velocity_2,
            set_desired_angular_motor_velocity_2,
            max_angular_motor_torque_2,
            set_max_angular_motor_torque_2);
    }
);

#[cfg(feature = "dim3")]
macro_rules! _revolute_motor_limit_methods (
    ($ty: ident, $revo: ident,
     $min_angle:         ident,
     $max_angle:         ident,
     $disable_min_angle: ident,
     $disable_max_angle: ident,
     $enable_min_angle:  ident,
     $enable_max_angle:  ident,
     $is_motor_enabled:  ident,
     $enable_motor:      ident,
     $disable_motor:     ident,
     $desired_motor_velocity:     ident,
     $set_desired_motor_velocity: ident,
     $max_motor_torque:           ident,
     $set_max_motor_torque:       ident
     ) => {
        impl<N: RealField> $ty<N> {
            /// The lower limit of the rotation angle.
            pub fn $min_angle(&self) -> Option<N> {
                self.$revo.min_angle()
            }

            /// The upper limit of the rotation angle.
            pub fn $max_angle(&self) -> Option<N> {
                self.$revo.max_angle()
            }

            /// Disable the lower limit of the rotation angle.
            pub fn $disable_min_angle(&mut self) {
                self.$revo.disable_max_angle();
            }

            /// Disable the upper limit of the rotation angle.
            pub fn $disable_max_angle(&mut self) {
                self.$revo.disable_max_angle();
            }

            /// Enable and set the lower limit of the rotation angle.
            pub fn $enable_min_angle(&mut self, limit: N) {
                self.$revo.enable_min_angle(limit);
            }

            /// Enable and set the upper limit of the rotation angle.
            pub fn $enable_max_angle(&mut self, limit: N) {
                self.$revo.enable_max_angle(limit)
            }

            /// Return `true` if the angular motor of this joint is enabled.
            pub fn $is_motor_enabled(&self) -> bool {
                self.$revo.is_angular_motor_enabled()
            }

            /// Enable the angular motor of this joint.
            pub fn $enable_motor(&mut self) {
                self.$revo.enable_angular_motor()
            }

            /// Disable the angular motor of this joint.
            pub fn $disable_motor(&mut self) {
                self.$revo.disable_angular_motor()
            }

            /// The desired angular velocity of the joint motor.
            pub fn $desired_motor_velocity(&self) -> N {
                self.$revo.desired_angular_motor_velocity()
            }

            /// Set the desired angular velocity of the joint motor.
            pub fn $set_desired_motor_velocity(&mut self, vel: N) {
                self.$revo.set_desired_angular_motor_velocity(vel)
            }

            /// The maximum torque that can be delivered by the joint motor.
            pub fn $max_motor_torque(&self) -> N {
                self.$revo.max_angular_motor_torque()
            }

            /// Set the maximum torque that can be delivered by the joint motor.
            pub fn $set_max_motor_torque(&mut self, torque: N) {
                self.$revo.set_max_angular_motor_torque(torque)
            }
        }
    }
);
