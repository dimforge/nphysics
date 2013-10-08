pub use object::rigid_body::{RigidBody, Static, Dynamic}; // FIXME: rename to StaticBody, DynamicBody ?
pub use object::soft_body::SoftBody;
pub use object::body::{Body, RB, SB};

pub mod rigid_body;
pub mod soft_body;
pub mod body;
pub mod volumetric;
