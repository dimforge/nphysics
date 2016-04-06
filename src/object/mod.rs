//! Rigid bodies.

mod rigid_body;
mod rigid_body_collision_groups;

pub use object::rigid_body::{RigidBody, RigidBodyHandle, ActivationState, RigidBodyState};
pub use object::rigid_body_collision_groups::RigidBodyCollisionGroups;
