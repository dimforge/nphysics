use ncollide::math::N;

pub trait Integrator<O> {
    fn add(&mut self, @mut O);
    fn remove(&mut self, @mut O);
    fn update(&mut self, N);
    fn priority(&self) -> f64;
}
