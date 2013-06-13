pub trait HasProxy<P>
{
  fn proxy<'l>(&'l self) -> &'l P;
  fn proxy_mut<'l>(&'l mut self) -> &'l mut P;
  fn set_proxy(&mut self, &P);
}
