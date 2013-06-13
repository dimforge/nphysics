use std::num::Zero;
use ncollide::contact::contact::Contact;
use ncollide::contact::geometric_contact::GeometricContact;
use constraint::contact_with_impulse::ContactWithImpulse;

pub struct GeometricContactWithImpulse<V, N>
{
  priv contact: GeometricContact<V, N>,
  priv impulse: N
}

impl<V: Copy + Neg<V>, N: Zero + Copy> Contact<V, N> for
GeometricContactWithImpulse<V, N>
{
    fn new(center: &V, normal: &V, depth: &N, world1: &V, world2: &V)
     -> GeometricContactWithImpulse<V, N>
  {
    GeometricContactWithImpulse {
      contact: Contact::new(center, normal, depth, world1, world2),
      impulse: Zero::zero()
    }
  }

  fn flip(&mut self)
  { self.contact.flip() }

  fn set_center(&mut self, center: &V)
  { self.contact.set_center(center) }

  fn center(&self) -> V
  { self.contact.center() }

  fn set_normal(&mut self, normal: &V)
  { self.contact.set_normal(normal) }

  fn normal(&self) -> V
  { self.contact.normal() }

  fn set_depth(&mut self, depth: &N)
  { self.contact.set_depth(depth) }

  fn depth(&self) -> N
  { self.contact.depth() }

  fn set_world1(&mut self, world1: &V)
  { self.contact.set_world1(world1) }

  fn world1(&self) -> V
  { self.contact.world1() }

  fn set_world2(&mut self, world2: &V)
  { self.contact.set_world2(world2) }

  fn world2(&self) -> V
  { self.contact.world2() }

}

impl<V, N: Copy> ContactWithImpulse<V, N> for GeometricContactWithImpulse<V, N>
{
  fn set_impulse(&mut self, impulse: N)
  { self.impulse = impulse; }

  fn impulse(&self) -> N
  { self.impulse }
}
