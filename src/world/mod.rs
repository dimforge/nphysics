//! The physics world.

pub use self::collider_world::{ColliderWorld, DefaultColliderWorld};
pub use self::dynamic_world::{DynamicWorld, DefaultDynamicWorld};

mod collider_world;
mod dynamic_world;
