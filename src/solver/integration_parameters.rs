use na::{self, Real};

pub struct IntegrationParameters<N: Real> {
    pub dt: N,
    pub erp: N,
    pub warmstart_coeff: N,
    pub max_velocity_iterations: usize,
    pub max_position_iterations: usize,
}

impl<N: Real> IntegrationParameters<N> {
    pub fn new(
        dt: N,
        erp: N,
        warmstart_coeff: N,
        max_velocity_iterations: usize,
        max_position_iterations: usize,
    ) -> Self {
        IntegrationParameters {
            dt,
            erp,
            warmstart_coeff,
            max_velocity_iterations,
            max_position_iterations,
        }
    }
}

impl<N: Real> Default for IntegrationParameters<N> {
    fn default() -> Self {
        Self::new(
            na::convert(1.0 / 60.0),
            na::convert(0.2),
            na::convert(0.5),
            10,
            3,
        )
    }
}
