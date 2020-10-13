//! The physics world.

pub use self::geometrical_world::{
    BroadPhasePairFilterSets, DefaultBroadPhasePairFilterSets, DefaultGeometricalWorld, GeometricalWorld,
};
pub use self::mechanical_world::{DefaultMechanicalWorld, MechanicalWorld};

mod geometrical_world;
mod mechanical_world;
