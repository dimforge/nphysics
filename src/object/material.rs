use na::{self, Real};

/// Description of the state of surface of a solid.
///
/// Strictly speaking, the coefficient provided here only exist
/// when considering a pair of touching surfaces. In practive, nphysics
/// will average the coefficient of the two surfaces in contact in order
/// to deduce the restitution/friction coefficient.
#[derive(Clone)]
pub struct Material<N: Real> {
    /// Restitution coefficient of the surface.
    pub restitution: N,
    /// Friction coefficient of the surface.
    pub friction: N,
}

impl<N: Real> Material<N> {
    /// Initialize a material with the specified restitution and friction coefficient.
    pub fn new(restitution: N, friction: N) -> Self {
        Material {
            restitution,
            friction,
        }
    }
}

impl<N: Real> Default for Material<N> {
    fn default() -> Self {
        Material::new(N::zero(), na::convert(0.5))
    }
}
