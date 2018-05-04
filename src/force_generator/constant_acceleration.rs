use na::Real;

use solver::IntegrationParameters;
use force_generator::ForceGenerator;
use object::{BodyHandle, BodySet};
use math::{Velocity, Vector};

/// Force generator adding a constant acceleration
/// at the center of mass of a set of body parts.
pub struct ConstantAcceleration<N: Real> {
    parts: Vec<BodyHandle>,
    acceleration: Velocity<N>,
}

impl<N: Real> ConstantAcceleration<N> {
    /// Adds a new constant acceleration generator.
    ///
    /// The acceleration is expressed in world-space.
    #[cfg(feature = "dim3")]
    pub fn new(linear_acc: Vector<N>, angular_acc: Vector<N>) -> Self {
        ConstantAcceleration {
            parts: Vec::new(),
            acceleration: Velocity::new(linear_acc, angular_acc),
        }
    }

    /// Adds a new constant acceleration generator.
    ///
    /// The acceleration is expressed in world-space.
    #[cfg(feature = "dim2")]
    pub fn new(linear_acc: Vector<N>, angular_acc: N) -> Self {
        ConstantAcceleration {
            parts: Vec::new(),
            acceleration: Velocity::new(linear_acc, angular_acc),
        }
    }

    /// Add a body part to be affected by this force generator.
    pub fn add_body_part(&mut self, body: BodyHandle) {
        self.parts.push(body)
    }
}

impl<N: Real> ForceGenerator<N> for ConstantAcceleration<N> {
    fn apply(&mut self, _: &IntegrationParameters<N>, bodies: &mut BodySet<N>) -> bool {
        let mut i = 0;

        while i < self.parts.len() {
            let body = self.parts[i];

            if bodies.contains(body) {
                let mut part = bodies.body_part_mut(body);
                let force = part.as_ref().inertia() * self.acceleration;
                part.apply_force(&force);
                i += 1;
            } else {
                let _ = self.parts.swap_remove(i);
            }
        }

        true
    }
}
