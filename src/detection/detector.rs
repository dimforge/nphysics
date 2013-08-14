pub trait Detector<N, O, I> {
    fn add(&mut self, @mut O);
    fn remove(&mut self, @mut O);
    fn activate(&mut self, @mut O, &mut ~[I]);
    fn deactivate(&mut self, @mut O);
    fn update(&mut self);
    fn interferences(&mut self, &mut ~[I]);

    fn interferences_array(&mut self) -> ~[I] {
        let mut res = ~[];

        self.interferences(&mut res);

        res
    }
}
