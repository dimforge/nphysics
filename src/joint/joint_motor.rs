use na::RealField;
use num::Zero;
use crate::solver::ImpulseLimits;

/// Description of a motor applied to a joint.
#[derive(Copy, Clone, Debug)]
pub struct JointMotor<V, N: RealField> {
    /// The velocity the motor will attempt to reach.
    pub desired_velocity: V,
    /// The maximum force deliverable by the motor.
    pub max_force: N,
    /// Whether or not the motor is active.
    pub enabled: bool,
}

impl<V: Zero, N: RealField> JointMotor<V, N> {
    /// Create a disable motor with zero desired velocity.
    ///
    /// The max force is initialized to a virtually infinite value, i.e., `N::max_value()`.
    pub fn new() -> Self {
        JointMotor {
            desired_velocity: V::zero(),
            max_force: N::max_value(),
            enabled: false,
        }
    }

    /// The limits of the impulse applicable by the motor on the body parts.
    pub fn impulse_limits(&self) -> ImpulseLimits<N> {
        ImpulseLimits::Independent {
            min: -self.max_force,
            max: self.max_force,
        }
    }
}

impl<V: Zero, N: RealField> Default for JointMotor<V, N> {
    fn default() -> Self {
        Self::new()
    }
}
