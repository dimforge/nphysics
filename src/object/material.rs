use na::{self, Real};

#[derive(Clone)]
pub struct Material<N: Real> {
    pub restitution: N,
    pub friction: N,
}

impl<N: Real> Material<N> {
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
