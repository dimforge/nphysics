use na::RealField;

use crate::solver::IntegrationParameters;
use crate::force_generator::ForceGenerator;
use crate::object::{BodyPartHandle, DefaultBodySet, BodySet, Body, BodyHandle};
use crate::math::{Force, ForceType, Velocity, Vector};

/// Force generator adding a constant acceleration
/// at the center of mass of a set of body parts.
pub struct ConstantAcceleration<N: RealField, Handle: BodyHandle> {
    parts: Vec<BodyPartHandle<Handle>>,
    acceleration: Velocity<N>,
}

impl<N: RealField, Handle: BodyHandle> ConstantAcceleration<N, Handle> {
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
    pub fn add_body_part(&mut self, body: BodyPartHandle<Handle>) {
        self.parts.push(body)
    }
}

impl<N: RealField, Handle: BodyHandle, Bodies: BodySet<N, Handle = Handle>> ForceGenerator<N, Bodies> for ConstantAcceleration<N, Handle> {
    fn apply(&mut self, _: &IntegrationParameters<N>, bodies: &mut Bodies) {
        let acceleration = self.acceleration;
        self.parts.retain(|h| {
            if let Some(body) = bodies.get_mut(h.0) {
                body.apply_force(h.1,
                                 &Force::new(acceleration.linear, acceleration.angular),
                                 ForceType::AccelerationChange,
                                  false);
                true
            } else {
                false
            }
        });
    }
}
