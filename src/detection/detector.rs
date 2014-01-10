pub trait Detector<O, I, BF> {
    fn update(&mut self, &mut BF);
    fn interferences(&mut self, &mut ~[I], &mut BF);
}
