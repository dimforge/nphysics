//! Collision detection and joints.

pub use self::activation_manager::ActivationManager;
pub use self::collider_contact_manifold::ColliderContactManifold;

mod collider_contact_manifold;
mod activation_manager;
