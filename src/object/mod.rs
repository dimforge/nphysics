//! Objects that may be added to the physical world.

pub use self::body::{ActivationStatus, Body, BodyPart, BodyStatus, BodyUpdateStatus};
pub use self::body_set::{Bodies, BodiesMut, BodyPartHandle, BodySet, BodyHandle, BodyDesc};
pub use self::collider::{Collider, ColliderData, ColliderAnchor, ColliderHandle, ColliderDesc, DeformableColliderDesc};
pub use self::ground::Ground;
pub use self::multibody::{Multibody, MultibodyDesc};
pub(crate) use self::multibody_link::MultibodyLinkVec;
pub use self::multibody_link::MultibodyLink;
pub use self::rigid_body::{RigidBody, RigidBodyDesc};
#[cfg(feature = "dim2")]
pub use self::fem_surface::{FEMSurface, FEMSurfaceDesc};
#[cfg(feature = "dim3")]
pub use self::fem_volume::{FEMVolume, FEMVolumeDesc};
pub use self::mass_constraint_system::{MassConstraintSystem, MassConstraintSystemDesc};
pub use self::mass_spring_system::{MassSpringSystem, MassSpringSystemDesc};
pub(crate) use self::fem_helper::FiniteElementIndices;

mod body;
mod body_set;
mod collider;
mod ground;
mod multibody;
mod multibody_link;
mod rigid_body;
#[cfg(feature = "dim2")]
mod fem_surface;
#[cfg(feature = "dim3")]
mod fem_volume;
mod mass_spring_system;
mod mass_constraint_system;
pub(crate) mod fem_helper;