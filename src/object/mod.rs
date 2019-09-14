//! Objects that may be added to the physical world.

pub use self::body::{
    ActivationStatus, Body, BodyPart, BodyPartMotion, BodyStatus, BodyUpdateStatus,
};
pub use self::body_set::{
    BodyDesc, BodyHandle, BodyPartHandle, BodySet, DefaultBodyHandle, DefaultBodyPartHandle,
    DefaultBodySet,
};
pub use self::collider::{
    Collider, ColliderAnchor, ColliderData, ColliderDesc, ColliderRemovalData,
    DeformableColliderDesc,
};
pub use self::collider_set::{
    ColliderHandle, ColliderSet, DefaultColliderHandle, DefaultColliderSet,
};
pub(crate) use self::fem_helper::FiniteElementIndices;
#[cfg(feature = "dim2")]
pub use self::fem_surface::{FEMSurface, FEMSurfaceDesc};
#[cfg(feature = "dim3")]
pub use self::fem_volume::{FEMVolume, FEMVolumeDesc};
pub use self::ground::Ground;
pub use self::mass_constraint_system::{MassConstraintSystem, MassConstraintSystemDesc};
pub use self::mass_spring_system::{MassSpringSystem, MassSpringSystemDesc};
pub use self::multibody::{Multibody, MultibodyDesc};
pub use self::multibody_link::MultibodyLink;
pub(crate) use self::multibody_link::MultibodyLinkVec;
pub use self::rigid_body::{RigidBody, RigidBodyDesc};

mod body;
mod body_set;
mod collider;
mod collider_set;
pub(crate) mod fem_helper;
#[cfg(feature = "dim2")]
mod fem_surface;
#[cfg(feature = "dim3")]
mod fem_volume;
mod ground;
mod mass_constraint_system;
mod mass_spring_system;
mod multibody;
mod multibody_link;
mod rigid_body;
