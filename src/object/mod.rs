//! Objects that may be added to the physical world.

pub use self::rigid_body_collision_groups::RigidBodyCollisionGroups;
pub use self::sensor_collision_groups::SensorCollisionGroups;
pub use self::collision_groups_wrapper_impl::{SENSOR_GROUP_ID, STATIC_GROUP_ID};

pub use self::ground::Ground;
pub use self::body::{ActivationStatus, Body, BodyMut, BodyPart, BodyStatus};
pub use self::body_set::{BodyHandle, BodySet};
pub use self::multibody_link::{MultibodyLink, MultibodyLinkId, MultibodyLinkMut, MultibodyLinkRef,
                               MultibodyLinkVec};
pub use self::rigid_body::RigidBody;
pub use self::multibody::{Multibody, MultibodyLinks, MultibodyWorkspace};
pub use self::collider::{Collider, ColliderData, ColliderHandle, Colliders, Sensor, SensorHandle};

mod collision_groups_wrapper_impl;
mod rigid_body_collision_groups;
mod sensor_collision_groups;

mod body;
mod body_set;
mod ground;
mod multibody;
mod multibody_link;
mod rigid_body;
mod collider;
