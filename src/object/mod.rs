//! Objects that may be added to the physical world.

pub use self::rigid_body::{RigidBody, ActivationState, RigidBodyState};
pub use self::sensor::{Sensor, sensor_handle_proximity};
pub use self::world_object::WorldObject;
pub use self::rigid_body_collision_groups::RigidBodyCollisionGroups;
pub use self::sensor_collision_groups::SensorCollisionGroups;
pub use self::collision_groups_wrapper_impl::{STATIC_GROUP_ID, SENSOR_GROUP_ID};

mod rigid_body;
mod sensor;
mod world_object;
mod collision_groups_wrapper_impl;
mod rigid_body_collision_groups;
mod sensor_collision_groups;
