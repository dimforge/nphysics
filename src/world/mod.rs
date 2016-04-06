//! The physics world.

// FIXME put this here ???
/// internal reserved static group number
pub const STATIC_GROUP_ID: usize = 29; // FIXME u32 please

mod world;
mod collision_groups;

pub use world::world::{World, WorldBroadPhase, RigidBodyCollisionWorld/*, RigidBodies*/};
pub use world::collision_groups::RigidBodyCollisionGroups;

