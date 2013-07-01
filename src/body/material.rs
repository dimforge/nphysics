pub trait Material<N>
{
  pub fn restitution_coefficient(&self) -> N;
  pub fn friction_coefficient(&self)    -> N; // FIXME: use a more realistic representation
}
