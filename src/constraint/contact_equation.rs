use std::num::{One, Zero, Orderable, Bounded};
use nalgebra::traits::basis::Basis;
use nalgebra::traits::cross::Cross;
use nalgebra::traits::dim::Dim;
use nalgebra::traits::division_ring::DivisionRing;
use nalgebra::traits::dot::Dot;
use nalgebra::traits::iterable::Iterable;
use nalgebra::traits::transformation::Transform;
use nalgebra::traits::translation::Translation;
use nalgebra::traits::vector_space::VectorSpace;
use ncollide::contact::contact::Contact;
use body::can_move::CanMove;
use body::dynamic::Dynamic;
use body::material::Material;
use constraint::index_proxy::HasIndexProxy;
use constraint::contact_with_impulses::ContactWithImpulses;

pub struct CorrectionParameters<N>
{
  depth_limit: N,
  corr_factor: N,
  depth_eps:   N,
  rest_eps:    N
}

pub struct ConstraintsSystemParameters<N>
{
  dt:                    N,
  num_first_order_iter:  uint,
  num_second_order_iter: uint,
  J:                     ~[N],
  MJ:                    ~[N],
  b:                     ~[N],
  lambda:                ~[N],
  bounds:                ~[N],
  idx:                   ~[int]
}

#[inline]
pub fn needs_first_order_resolution<C: ContactWithImpulses<V, N>, V, N: Ord>
       (c: &C, depth_limit: &N) -> bool
{ c.depth() > *depth_limit }

pub fn fill_first_order_contact_equation
       <C:  Contact<LV, N>,
        LV: Iterable<N> + VectorSpace<N> + Cross<AV> + Dim, // FIXME: is Dim safe?
        AV: Iterable<N> + VectorSpace<N> + Dim, // FIXME: is Dim safe?
        N:  DivisionRing + Orderable + Bounded + Clone,
        RB: Dynamic<N, LV, AV, II> + Translation<LV> + HasIndexProxy + CanMove,
        II: Transform<AV>>
       (coll:        &C,
        rb1:         &RB,
        rb2:         &RB,
        system:      &mut ConstraintsSystemParameters<N>,
        ididx:       &mut uint,
        idJ:         &mut uint,
        idb:         &mut uint,
        idbounds:    &mut uint,
        correction:  &CorrectionParameters<N>)
{
  let _LV_sz = Dim::dim::<LV>();
  let _AV_sz = Dim::dim::<AV>();

  /*
   * Fill J and MJ
   */
  fill_M_MJ(-coll.normal(), coll.center(), rb1, system, *idJ);
  fill_M_MJ(coll.normal(), coll.center(), rb2, system, *idJ + _LV_sz + _AV_sz);

  *idJ = *idJ + 2 * (_AV_sz + _LV_sz);

  /*
   * Fill idx
   */
  set_idx(ididx, system.idx, rb1, rb2);

  /*
   * Fill b
   */
  system.b[*idb] = correction.corr_factor * (coll.depth() -
                                             correction.depth_limit).max(&Zero::zero()) / system.dt;

  /*
   * Reset forces
   */
  system.lambda[*idb] = Zero::zero();

  *idb = *idb + 1;

  /*
   * Fill bounds
   */
  set_contact_bounds(idbounds, system.bounds);
}

 // FIXME: note that removing the Clone constraint on N leads to an ICE
pub fn fill_second_order_contact_equation
       <C:  ContactWithImpulses<LV, N>,
        LV: Iterable<N> + VectorSpace<N> + Cross<AV>  + Dot<N> + Dim +
            Basis + Clone,
        AV: Iterable<N> + VectorSpace<N> + Dot<N> + Dim,
        N:  DivisionRing + Orderable + Bounded + Clone + ToStr,
        RB: Dynamic<N, LV, AV, II> + Translation<LV> + HasIndexProxy +
            CanMove + Material<N>,
        II: Transform<AV>>
       (coll:       &C,
        rb1:        &RB,
        rb2:        &RB,
        system:     &mut ConstraintsSystemParameters<N>,
        ididx:      &mut uint,
        idJ:        &mut uint,
        idb:        &mut uint,
        idbounds:   &mut uint,
        correction: &CorrectionParameters<N>)
{
  let _2 = One::one::<N>() + One::one::<N>(); // FIXME: bad

  let err         = correction.corr_factor *
                    (coll.depth().min(&correction.depth_limit) - correction.depth_eps).max(&Zero::zero())
                    / system.dt;
  let restitution = (rb1.restitution_coefficient() + rb2.restitution_coefficient()) / _2;

  fill_velocity_constraint(coll.normal(),
                           coll.center(),
                           restitution,
                           err,
                           Zero::zero(), // coll.impulses()[0].clone(),
                           Zero::zero(),
                           Bounded::max_value(),
                           correction.rest_eps.clone(),
                           rb1, rb2, system, ididx, idJ,
                           idb, idbounds);


  let mean_friction = (rb1.friction_coefficient() + rb2.friction_coefficient()) / _2;
  let mut i = 1;
  do coll.normal().orthonormal_subspace_basis() |friction_axis|
  {
    fill_velocity_constraint(friction_axis,
                             coll.center(),
                             Zero::zero(),
                             Zero::zero(),
                             Zero::zero(), // coll.impulses()[i].clone(),
                             -mean_friction * coll.impulses()[0],
                             mean_friction * coll.impulses()[0],
                             correction.rest_eps.clone(),
                             rb1, rb2, system, ididx, idJ,
                             idb, idbounds);
    i = i + 1;
  }
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

fn fill_M_MJ<LV: Iterable<N> + VectorSpace<N> + Cross<AV> + Dim,
             AV: Iterable<N> + VectorSpace<N>,
             N:  DivisionRing + Clone,
             RB: Dynamic<N, LV, AV, II> + HasIndexProxy +
                 Translation<LV> + CanMove,
             II: Transform<AV>>
   (normal: LV,
    center: LV,
    rb:     &RB,
    system: &mut ConstraintsSystemParameters<N>,
    idJ:    uint) -> AV
{
  if rb.can_move()
  {
    let mut idJb = idJ;

    // rotation axis
    let rot_axis = (center - rb.translation()).cross(&normal);

    // dump everything
    for normal.iter().enumerate().advance |(i, e)|
    { system.J[idJb + i] = e.clone() }

    for Dim::dim::<LV>().times
    {
      system.MJ[idJb] = system.J[idJb] * rb.inv_mass();
      idJb = idJb + 1;
    }

    for rot_axis.iter().enumerate().advance |(i, e)|
    { system.J[idJb + i] = e.clone() }

    // rotation momentum
    let w_rot_axis = rb.inv_inertia().transform_vec(&rot_axis);
    for w_rot_axis.iter().enumerate().advance |(i, e)|
    { system.MJ[idJb + i] = e.clone() }

    rot_axis
  }
  else
  { Zero::zero() }
}

fn fill_velocity_constraint
       <LV: Iterable<N> + VectorSpace<N> + Cross<AV>  + Dot<N> + Dim +
            Basis + Clone,
        AV: Iterable<N> + VectorSpace<N> + Dot<N> + Dim,
        N:  DivisionRing + Orderable + Bounded + Clone,
        RB: Dynamic<N, LV, AV, II> + Translation<LV> + HasIndexProxy +
            CanMove,
        II: Transform<AV>>
       (normal:          LV,
        center:          LV,
        restitution:     N,
        error:           N,
        initial_impulse: N,
        hibound:         N,
        lobound:         N,
        rest_eps:        N,
        rb1:             &RB,
        rb2:             &RB,
        system:          &mut ConstraintsSystemParameters<N>,
        ididx:           &mut uint,
        idJ:             &mut uint,
        idb:             &mut uint,
        idbounds:        &mut uint)
{
  let _LV_sz = Dim::dim::<LV>();
  let _AV_sz = Dim::dim::<AV>();

  /*
   * Fill J and MJ
   */
  let rot_axis1 = fill_M_MJ(-normal, center.clone(), rb1, system, *idJ);
  let rot_axis2 = fill_M_MJ(normal.clone(), center.clone(), rb2, system, *idJ + _LV_sz + _AV_sz);

  *idJ = *idJ + 2 * (_AV_sz + _LV_sz);

  /*
   * Fill idx
   */
  set_idx(ididx, system.idx, rb1, rb2);

  /*
   * Fill b
   */
  let idbv = *idb;

  system.b[idbv] = Zero::zero();

  if rb1.can_move()
  {
    system.b[idbv] = system.b[idbv] +
      - (rb1.lin_vel() + rb1.ext_lin_force().scalar_mul(&system.dt)).dot(&normal)
      + (rb1.ang_vel() + rb1.ext_ang_force().scalar_mul(&system.dt)).dot(&rot_axis1);
  }

  if rb2.can_move()
  {
    system.b[idbv] = system.b[idbv] +
        (rb2.lin_vel() + rb2.ext_lin_force().scalar_mul(&system.dt)).dot(&normal)
      + (rb2.ang_vel() + rb2.ext_ang_force().scalar_mul(&system.dt)).dot(&rot_axis2);
  }

  if system.b[idbv] > rest_eps
  {
    system.b[idbv] = system.b[idbv] + restitution * system.b[idbv]
  }

  system.b[idbv] = error - system.b[idbv];

  system.lambda[idbv] = initial_impulse;

  *idb = *idb + 1;

  /*
   * Fill bounds
   */
  set_constraints_bounds(idbounds, system.bounds, hibound, lobound);
}
