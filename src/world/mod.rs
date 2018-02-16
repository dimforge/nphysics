//! The physics world.

pub use world::world::{World, WorldBroadPhase, RigidBodies, Sensors, RigidBodyCollisionWorld,
                       WorldCollisionObject, RigidBodyStorage, SensorStorage};

mod world;
