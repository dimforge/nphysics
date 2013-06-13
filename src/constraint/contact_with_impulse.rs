use ncollide::contact::contact::Contact;

pub trait ContactWithImpulse<V, N> : Contact<V, N>
{
  fn set_impulse(&mut self, N);
  fn impulse(&self) -> N;
  // FIXME: make distinction between normal impulse and friction impulse
}
