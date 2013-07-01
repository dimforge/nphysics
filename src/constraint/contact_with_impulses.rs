use ncollide::contact::contact::Contact;

pub trait ContactWithImpulses<V, N> : Contact<V, N>
{
  // FIXME: is it possible to avoid the ~ on the return type?
  fn impulses_mut<'r>(&'r mut self) -> &'r mut ~[N];
  fn impulses<'r>(&'r self)         -> &'r ~[N];
}
