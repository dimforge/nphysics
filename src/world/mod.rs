//! The physics world.

pub use self::world::World;
pub use self::collider_world::ColliderWorld;
pub use self::dynamic_world::DynamicWorld;

mod world;
mod collider_world;
mod dynamic_world;
