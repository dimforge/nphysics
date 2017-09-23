pub trait Storage<B> {
    type Index: Clone + ::std::hash::Hash + Eq;

    fn get(&self, index: &Self::Index) -> &B;
    fn get_mut(&mut self, index: &Self::Index) -> &mut B;
}
