use ncollide::math::N;

pub trait Integrator<O> {
    fn update(&mut self, N, &mut O);
}
