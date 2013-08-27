pub trait Detector<N, O, I> {
    fn add(&mut self, @mut O);
    fn remove(&mut self, @mut O);
    fn update(&mut self);
    fn interferences(&mut self, &mut ~[I]);
    fn priority(&self) -> f64;
}
