use num::Zero;
use na::Real;
use solver::ImpulseLimits;

#[derive(Copy, Clone, Debug)]
pub struct JointMotor<V, N: Real> {
    pub desired_velocity: V,
    pub max_force: N,
    pub enabled: bool,
}

impl<V: Zero, N: Real> JointMotor<V, N> {
    pub fn new() -> Self {
        JointMotor {
            desired_velocity: V::zero(),
            max_force: N::max_value(),
            enabled: false,
        }
    }

    pub fn impulse_limits(&self) -> ImpulseLimits<N> {
        ImpulseLimits::Independent {
            min: -self.max_force,
            max: self.max_force,
        }
    }
}
