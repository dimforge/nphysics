use na::{self, Real};

pub struct IntegrationParameters<N: Real> {
    pub dt: N,
    pub erp: N,
    pub warmstart_coeff: N,
    pub restitution_velocity_threshold: N,
    pub allowed_linear_error: N,
    pub allowed_angular_error: N,
    pub max_linear_correction: N,
    pub max_angular_correction: N,
    pub max_velocity_iterations: usize,
    pub max_position_iterations: usize,
}

impl<N: Real> IntegrationParameters<N> {
    pub fn new(
        dt: N,
        erp: N,
        warmstart_coeff: N,
        restitution_velocity_threshold: N,
        allowed_linear_error: N,
        allowed_angular_error: N,
        max_linear_correction: N,
        max_angular_correction: N,
        max_velocity_iterations: usize,
        max_position_iterations: usize,
    ) -> Self {
        IntegrationParameters {
            dt,
            erp,
            warmstart_coeff,
            restitution_velocity_threshold,
            allowed_linear_error,
            allowed_angular_error,
            max_linear_correction,
            max_angular_correction,
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
            na::convert(1.0),
            na::convert(1.0),
            na::convert(0.001),
            na::convert(0.001),
            na::convert(0.2),
            na::convert(0.2),
            8,
            3,
        )
    }
}
