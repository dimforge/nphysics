use na::RealField;

use crate::material::{Material, MaterialCombineMode, MaterialContext, LocalMaterialProperties};
use crate::math::Vector;


/// Description of the state of surface of a solid.
///
/// Strictly speaking, the coefficient provided here only exist
/// when considering a pair of touching surfaces. In practice, nphysics
/// will average the coefficient of the two surfaces in contact in order
/// to deduce the restitution/friction coefficient.
#[derive(Copy, Clone, Debug)]
pub struct BasicMaterial<N: RealField> {
    /// The ID of this material for automatic lookup.
    pub id: Option<u32>,
    /// Restitution coefficient of the surface.
    pub restitution: N,
    /// Friction coefficient of the surface.
    pub friction: N,
    /// The fictitious velocity at the surface of this material.
    pub surface_velocity: Option<Vector<N>>,
    /// The way restitution coefficients are combined if no match
    /// was found in the material lookup tables.
    pub restitution_combine_mode: MaterialCombineMode,
    /// The way friction coefficients are combined if no match
    /// was found in the material lookup tables.
    pub friction_combine_mode: MaterialCombineMode,
}


impl<N: RealField> BasicMaterial<N> {
    /// Initialize a material with the specified restitution and friction coefficients.
    pub fn new(restitution: N, friction: N) -> Self {
        BasicMaterial {
            id: None,
            restitution,
            friction,
            surface_velocity: None,
            restitution_combine_mode: MaterialCombineMode::Average,
            friction_combine_mode: MaterialCombineMode::Average
        }
    }
}

impl<N: RealField> Material<N> for BasicMaterial<N> {
    fn local_properties(&self, context: MaterialContext<N>) -> LocalMaterialProperties<N> {
        LocalMaterialProperties {
            id: self.id,
            restitution: (self.restitution, self.restitution_combine_mode),
            friction: (self.friction, self.friction_combine_mode),
            surface_velocity: self.surface_velocity.map(|v| context.collider.position() * v).unwrap_or(Vector::zeros()),
        }
    }
}

impl<N: RealField> Default for BasicMaterial<N> {
    fn default() -> Self {
        BasicMaterial::new(N::zero(), na::convert(0.5))
    }
}