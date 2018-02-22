//! Collision detection and joints.

pub use self::activation_manager::ActivationManager;
pub use self::body_contact_manifold::BodyContactManifold;

mod body_contact_manifold;
mod activation_manager;
