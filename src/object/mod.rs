//! Objects that may be added to the physical world.

pub use self::material::Material;

pub use self::body::{ActivationStatus, Body, BodyPart, BodyStatus};
pub use self::body_set::{Bodies, BodiesMut, BodyPartHandle, BodySet, BodyHandle};
pub use self::collider::{Collider, ColliderData, ColliderAnchor, ColliderHandle, Colliders, Sensor, SensorHandle};
pub use self::ground::Ground;
pub use self::multibody::{Multibody, MultibodyLinks, MultibodyWorkspace};
pub(crate) use self::multibody_link::MultibodyLinkVec;
pub use self::multibody_link::MultibodyLink;
pub use self::rigid_body::RigidBody;
#[cfg(feature = "dim3")]
pub use self::deformable_volume::DeformableVolume;
#[cfg(feature = "dim3")] // FIXME: allow in 2D too.
pub use self::mass_spring_surface::MassSpringSurface;
#[cfg(feature = "dim3")] // FIXME: allow in 2D too.
pub use self::mass_constraint_surface::MassConstraintSurface;

mod material;

mod body;
mod body_set;
mod collider;
mod ground;
mod multibody;
mod multibody_link;
mod rigid_body;
#[cfg(feature = "dim3")]
mod deformable_volume;
#[cfg(feature = "dim3")]
mod mass_spring_surface;
#[cfg(feature = "dim3")]
mod mass_constraint_surface;