//! The physics world.

pub use self::geometrical_world::{GeometricalWorld, DefaultGeometricalWorld};
pub use self::mechanical_world::{MechanicalWorld, DefaultMechanicalWorld};

mod geometrical_world;
mod mechanical_world;
