pub trait HasIndexProxy
{
  // FIXME: use those or directy provide "index" and "index_mut" methods?
  fn proxy<'r>(&'r self)         -> &'r IndexProxy;
  fn proxy_mut<'r>(&'r mut self) -> &'r mut IndexProxy;
}

#[deriving(ToStr)]
pub struct IndexProxy
{ index: int }

impl IndexProxy
{
  pub fn new() -> IndexProxy
  { IndexProxy { index: 0 } }
}
