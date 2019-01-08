//! Objects that may be added to the physical world.

pub use self::material::Material;

pub use self::body::{ActivationStatus, Body, BodyPart, BodyStatus};
pub use self::body_set::{Bodies, BodiesMut, BodyPartHandle, BodySet, BodyHandle, BodyDesc};
pub use self::collider::{Collider, ColliderData, ColliderAnchor, ColliderHandle, ColliderDesc, DeformableColliderDesc};
pub use self::ground::Ground;
pub use self::multibody::{Multibody, MultibodyDesc};
pub(crate) use self::multibody_link::MultibodyLinkVec;
pub use self::multibody_link::MultibodyLink;
pub use self::rigid_body::{RigidBody, RigidBodyDesc};
#[cfg(feature = "dim2")]
pub use self::deformable_surface::{DeformableSurface, DeformableSurfaceDesc};
#[cfg(feature = "dim3")]
pub use self::deformable_volume::{DeformableVolume, DeformableVolumeDesc};
pub use self::mass_constraint_system::MassConstraintSystem;
pub use self::mass_spring_system::MassSpringSystem;
pub use self::fem_helper::FiniteElementIndices;

mod material;

mod body;
mod body_set;
mod collider;
mod ground;
mod multibody;
mod multibody_link;
mod rigid_body;
#[cfg(feature = "dim2")]
mod deformable_surface;
#[cfg(feature = "dim3")]
mod deformable_volume;
mod mass_spring_system;
mod mass_constraint_system;
pub(crate) mod fem_helper;