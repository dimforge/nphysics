//! Objects that may be added to the physical world.

pub use self::body::{ActivationStatus, Body, BodyPart, BodyStatus, BodyUpdateStatus, BodyPartMotion};
pub use self::body_set::{
    BodyPartHandle, DefaultBodySet, BodySet,
    DefaultBodyHandle, DefaultBodyPartHandle, BodyDesc, BodyHandle
};
pub use self::collider::{Collider, ColliderData, ColliderAnchor, ColliderDesc, DeformableColliderDesc, ColliderRemovalData};
pub use self::collider_set::{ColliderSet, ColliderHandle, DefaultColliderSet, DefaultColliderHandle};
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
pub use self::pbf_fluid::PBFFluid;
pub use self::iisph_fluid::IISPHFluid;
pub use self::sph_kernel::{SPHKernel, SpikyKernel, CubicSplineKernel, ViscosityKernel, Poly6Kernel};
pub(crate) use self::fem_helper::FiniteElementIndices;

mod body;
mod body_set;
mod collider;
mod collider_set;
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
mod pbf_fluid;
mod iisph_fluid;
mod sph_kernel;
pub(crate) mod fem_helper;