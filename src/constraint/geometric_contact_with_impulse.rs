use std::num::Zero;
use std::vec;
use nalgebra::traits::dim::Dim;
use ncollide::contact::contact::Contact;
use ncollide::contact::contact::UpdatableContact;
use ncollide::contact::geometric_contact::GeometricContact;
use constraint::contact_with_impulses::ContactWithImpulses;

#[deriving(ToStr, Clone, Eq)]
pub struct GeometricContactWithImpulse<V, N>
{
  priv contact:  GeometricContact<V, N>,
  priv impulses: ~[N]
}

// FIXME: the compiler does not infer correctly the DeepClone impl.
impl<V: DeepClone, N: DeepClone> DeepClone for GeometricContactWithImpulse<V, N>
{
  fn deep_clone(&self) -> GeometricContactWithImpulse<V, N>
  {
    GeometricContactWithImpulse {
      contact: self.contact.deep_clone(),
      impulses: self.impulses.map(|e| e.deep_clone())
    }
  }
}

impl<V: Clone + Neg<V> + Zero + Dim, N: Zero + Clone> Contact<V, N> for
GeometricContactWithImpulse<V, N>
{
  #[inline]
  fn new(center: V, normal: V, depth: N, world1: V, world2: V)
     -> GeometricContactWithImpulse<V, N>
  {
    GeometricContactWithImpulse {
      contact:  Contact::new(center, normal, depth, world1, world2),
      impulses: vec::from_elem(Dim::dim::<V>(), Zero::zero())
    }
  }

  #[inline]
  fn flip(&mut self)
  { self.contact.flip() }

  #[inline]
  fn set_center(&mut self, center: V)
  { self.contact.set_center(center) }

  #[inline]
  fn center(&self) -> V
  { self.contact.center() }

  #[inline]
  fn set_normal(&mut self, normal: V)
  { self.contact.set_normal(normal) }

  #[inline]
  fn normal(&self) -> V
  { self.contact.normal() }

  #[inline]
  fn set_depth(&mut self, depth: N)
  { self.contact.set_depth(depth) }

  #[inline]
  fn depth(&self) -> N
  { self.contact.depth() }

  #[inline]
  fn set_world1(&mut self, world1: V)
  { self.contact.set_world1(world1) }

  #[inline]
  fn world1(&self) -> V
  { self.contact.world1() }

  #[inline]
  fn set_world2(&mut self, world2: V)
  { self.contact.set_world2(world2) }

  #[inline]
  fn world2(&self) -> V
  { self.contact.world2() }
}

impl<V: Clone + Neg<V> + Dim, N: Zero + Clone> UpdatableContact<V, N> for
GeometricContactWithImpulse<V, N>
{
  #[inline]
  fn set_local1(&mut self, local1: V)
  { self.contact.set_local1(local1) }

  #[inline]
  fn local1(&self) -> V
  { self.contact.local1() }

  #[inline]
  fn set_local2(&mut self, local2: V)
  { self.contact.set_local2(local2) }

  #[inline]
  fn local2(&self) -> V
  { self.contact.local2() }
}

impl<V, N: Clone> ContactWithImpulses<V, N> for GeometricContactWithImpulse<V, N>
{
  #[inline]
  fn impulses_mut<'r>(&'r mut self) -> &'r mut ~[N]
  { &'r mut self.impulses }

  #[inline]
  fn impulses<'r>(&'r self) -> &'r ~[N]
  { &'r self.impulses }
}
