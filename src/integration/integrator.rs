pub trait Integrator<N, O> {
    fn add(&mut self, @mut O);
    fn remove(&mut self, @mut O);
    fn activate(&mut self, @mut O);
    fn deactivate(&mut self, @mut O);
    fn pre_update(&mut self, N);  // FIXME: find a better name
    fn post_update(&mut self, N); // FIXME: find a better name
}
