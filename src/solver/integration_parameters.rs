use na::{self, Real};

pub struct IntegrationParameters<N: Real> {
    pub dt: N,
    pub erp: N,
    pub niter: usize,
}

impl<N: Real> IntegrationParameters<N> {
    pub fn new(dt: N, erp: N, niter: usize) -> Self {
        IntegrationParameters { dt, erp, niter }
    }
}

impl<N: Real> Default for IntegrationParameters<N> {
    fn default() -> Self {
        Self::new(na::convert(1.0 / 60.0), na::convert(0.2), 10)
    }
}
