use std::num::{Zero, Orderable, Bounded};
use nalgebra::traits::dim::Dim;
use nalgebra::traits::vector_space::VectorSpace;
use nalgebra::traits::division_ring::DivisionRing;
use nalgebra::traits::iterable::Iterable;
use nalgebra::traits::translation::Translation;
use nalgebra::traits::transformation::Transform;
use nalgebra::traits::cross::Cross;
use nalgebra::traits::dot::Dot;
use ncollide::contact::contact::Contact;
use body::dynamic::Dynamic;
use body::can_move::CanMove;
use constraint::index_proxy::HasIndexProxy;
use constraint::contact_with_impulse::ContactWithImpulse;

pub fn needs_first_order_resolution<C: ContactWithImpulse<V, N>, V, N: Ord>
       (c: &C, depth_limit: N) -> bool
{ c.depth() > depth_limit }

pub fn fill_first_order_contact_equation
       <C:  Contact<LV, N>,
        LV: Iterable<N> + VectorSpace<N> + Cross<AV> + Dim, // FIXME: is Dim safe?
        AV: Iterable<N> + VectorSpace<N> + Dim, // FIXME: is Dim safe?
        N:  DivisionRing + Orderable + Bounded + Copy,
        RB: Dynamic<N, LV, AV, II> + Translation<LV> + HasIndexProxy +
            CanMove,
        II: Transform<AV>>
       (dt:          N,
        coll:        &C,
        rb1:         &RB,
        rb2:         &RB,
        J:           &mut [N],
        MJ:          &mut [N],
        b:           &mut [N],
        lambda:      &mut [N],
        bounds:      &mut [N],
        idx:         &mut [int],
        ididx:       &mut uint,
        idJ:         &mut uint,
        idb:         &mut uint,
        idbounds:    &mut uint,
        depth_limit: N,
        corr_factor: N)
{
  let _LV_sz = Dim::dim::<LV>();
  let _AV_sz = Dim::dim::<AV>();

  /*
   * Fill J and MJ
   */
  fill_M_MJ(coll, rb1, J, MJ, *idJ, true);
  fill_M_MJ(coll, rb2, J, MJ, *idJ + _LV_sz + _AV_sz, false);

  *idJ = *idJ + 2 * (_AV_sz + _LV_sz);

  /*
   * Fill idx
   */
  set_idx(ididx, idx, rb1, rb2);

  /*
   * Fill b
   */
  b[*idb] = corr_factor * (coll.depth() - depth_limit).max(&Zero::zero()) / dt;

  /*
   * Reset forces
   */
  lambda[*idb] = Zero::zero();

  *idb = *idb + 1;

  /*
   * Fill bounds
   */
  set_contact_bounds(idbounds, bounds);
}

 // FIXME: note that removing the Copy constraint on N leads to an ICE
pub fn fill_second_order_contact_equation
       <C:  ContactWithImpulse<LV, N>,
        LV: Iterable<N> + VectorSpace<N> + Cross<AV>  + Dot<N> + Dim,
        AV: Iterable<N> + VectorSpace<N> + Dot<N> + Dim,
        N:  DivisionRing + Orderable + Bounded + Copy,
        RB: Dynamic<N, LV, AV, II> + Translation<LV> + HasIndexProxy +
            CanMove,
        II: Transform<AV>>
       (dt:          N,
        coll:        &C,
        rb1:         &RB,
        rb2:         &RB,
        J:           &mut [N],
        MJ:          &mut [N],
        b:           &mut [N],
        lambda:      &mut [N],
        bounds:      &mut [N],
        idx:         &mut [int],
        ididx:       &mut uint,
        idJ:         &mut uint,
        idb:         &mut uint,
        idbounds:    &mut uint,
        depth_limit: N,
        corr_factor: N,
        depth_eps:   N)
{
  let _LV_sz = Dim::dim::<LV>();
  let _AV_sz = Dim::dim::<AV>();

  /*
   * Fill J and MJ
   */
  let rot_axis1 = fill_M_MJ(coll, rb1, J, MJ, *idJ, true);
  let rot_axis2 = fill_M_MJ(coll, rb2, J, MJ, *idJ + _LV_sz + _AV_sz, false);

  *idJ = *idJ + 2 * (_AV_sz + _LV_sz);

  /*
   * Fill idx
   */
  set_idx(ididx, idx, rb1, rb2);

  /*
   * Fill b
   */
  let idbv = *idb;

  b[idbv] = Zero::zero();

  if rb1.can_move()
  {
    b[idbv] = b[idbv] +
      - (rb1.lin_vel() + rb1.ext_lin_force().scalar_mul(&dt)).dot(&coll.normal())
      + (rb1.ang_vel() + rb1.ext_ang_force().scalar_mul(&dt)).dot(&rot_axis1);
  }

  if rb2.can_move()
  {
    b[idbv] = b[idbv] +
        (rb2.lin_vel() + rb2.ext_lin_force().scalar_mul(&dt)).dot(&coll.normal())
      + (rb2.ang_vel() + rb2.ext_ang_force().scalar_mul(&dt)).dot(&rot_axis2);
  }

  b[idbv] = b[idbv] - corr_factor * (coll.depth().min(&depth_limit) - depth_eps).max(&Zero::zero()) / dt;
  b[idbv] = -b[idbv];

  lambda[idbv] = coll.impulse();

  *idb = *idb + 1;

  /*
   * Fill bounds
   */
  set_contact_bounds(idbounds, bounds);
}

fn set_idx<RB: HasIndexProxy>(ididx: &mut uint,
                              idx:   &mut [int],
                              rb1:   &RB,
                              rb2:   &RB)
{
  idx[*ididx] = rb1.proxy().index;
  *ididx = *ididx + 1;
  idx[*ididx] = rb2.proxy().index;
  *ididx = *ididx + 1;
}

fn set_contact_bounds<N: Zero + Bounded>(idbounds: &mut uint, bounds: &mut [N])
{
  set_constraints_bounds(idbounds, bounds, Zero::zero(), Bounded::max_value());
}

fn set_constraints_bounds<N>(idbounds: &mut uint, bounds: &mut [N], lo: N, hi: N)
{
  bounds[*idbounds] = lo;
  *idbounds = *idbounds + 1;
  bounds[*idbounds] = hi;
  *idbounds = *idbounds + 1;
}

fn fill_M_MJ<C:  Contact<LV, N>,
             LV: Iterable<N> + VectorSpace<N> + Cross<AV> + Dim,
             AV: Iterable<N> + VectorSpace<N>,
             N:  DivisionRing + Copy,
             RB: Dynamic<N, LV, AV, II> + HasIndexProxy +
                 Translation<LV> + CanMove,
             II: Transform<AV>>
   (coll: &C, rb: &RB, J: &mut [N], MJ: &mut [N], idJ: uint, neg: bool) -> AV
{
  if rb.can_move()
  {
    let mut idJb = idJ;

    // normal
    let normal = if neg { -coll.normal() } else { coll.normal() };

    // rotation axis
    let rot_axis = (coll.center() - rb.translation()).cross(&normal);

    // dump everything
    for normal.iter().enumerate().advance |(i, e)|
    { J[idJb + i] = copy *e }

    for Dim::dim::<LV>().times
    {
      MJ[idJb] = J[idJb] * rb.inv_mass();
      idJb = idJb + 1;
    }

    for rot_axis.iter().enumerate().advance |(i, e)|
    { J[idJb + i] = copy *e }

    // rotation momentum
    let w_rot_axis = rb.inv_inertia().transform_vec(&rot_axis);
    for w_rot_axis.iter().enumerate().advance |(i, e)|
    { MJ[idJb + i] = copy *e }

    rot_axis
  }
  else
  { Zero::zero() }
}
