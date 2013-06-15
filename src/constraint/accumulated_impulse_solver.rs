use std::vec;
use std::num::{Zero, One, Orderable, Bounded};
use nalgebra::traits::vector_space::VectorSpace;
use nalgebra::traits::division_ring::DivisionRing;
use nalgebra::traits::flatten::Flatten;
use nalgebra::traits::translation::Translation;
use nalgebra::traits::rotation;
use nalgebra::traits::rotation::Rotation;
use nalgebra::traits::cross::Cross;
use nalgebra::traits::dot::Dot;
use nalgebra::traits::rlmul::RMul;
use ncollide::utils::has_proxy::HasProxy;
use body::dynamic::Dynamic;
use body::transformable::Transformable;
use body::can_move::CanMove;
use constraint::index_proxy::IndexProxy;
use constraint::contact_with_impulse::ContactWithImpulse;
use constraint::contact_equation;
use constraint::constraint_solver::ConstraintSolver;
use pgs = constraint::projected_gauss_seidel_solver;


pub struct AccumulatedImpulseSolver<N, C, LV, AV, RB, II, M>
{
  priv depth_limit:           N,
  priv corr_factor:           N,
  priv depth_eps:             N,
  priv num_first_order_iter:  uint,
  priv num_second_order_iter: uint,
  priv J:                     ~[N],
  priv MJ:                    ~[N],
  priv b:                     ~[N],
  priv lambda:                ~[N],
  priv bounds:                ~[N],
  priv idx:                   ~[int]
}

impl<C:  ContactWithImpulse<LV, N>,
     LV: Flatten<N> + VectorSpace<N> + Cross<AV>  + Dot<N>,
     AV: Flatten<N> + VectorSpace<N> + Dot<N>,
     N:  DivisionRing + Orderable + Bounded + Copy,
     RB: Dynamic<N, LV, AV, II> + Translation<LV> + HasProxy<IndexProxy> +
         Transformable<M> + CanMove,
     II: RMul<AV>,
     M: Translation<LV> + Rotation<AV> + One>
    AccumulatedImpulseSolver<N, C, LV, AV, RB, II, M>
{
  pub fn new(depth_limit:           N,
             corr_factor:           N,
             depth_eps:             N,
             num_first_order_iter:  uint,
             num_second_order_iter: uint) ->
    AccumulatedImpulseSolver<N, C, LV, AV, RB, II, M>
  {
    AccumulatedImpulseSolver {
      depth_limit: depth_limit,
      corr_factor: corr_factor,
      depth_eps:   depth_eps,
      num_first_order_iter:  num_first_order_iter,
      num_second_order_iter: num_second_order_iter,
      J:           ~[],
      MJ:          ~[],
      b:           ~[],
      lambda:      ~[],
      bounds:      ~[],
      idx:         ~[]
    }
  }

  fn resize_buffers(&mut self, num_equations: uint)
  {
    let _0      = Zero::zero::<N>();
    let max_dim = Flatten::flat_size::<N, LV>() + Flatten::flat_size::<N, AV>();

    vec::grow_set(&mut self.J, num_equations * 2 * max_dim, &_0, _0);
    vec::grow_set(&mut self.MJ, num_equations * 2 * max_dim, &_0, _0);
    vec::grow_set(&mut self.b, num_equations, &_0, _0);
    vec::grow_set(&mut self.lambda, num_equations, &_0, _0);
    vec::grow_set(&mut self.bounds, num_equations * 2, &_0, _0);
    vec::grow_set(&mut self.idx, num_equations * 2, &0, 0);
  }

  fn first_order_solve(&mut self,
                       dt:     N,
                       island: &[(@mut RB, @mut RB, @mut C)],
                       bodies: &[@mut RB])
  {
    let equations_per_contacts = 1;
    let num_equations          = equations_per_contacts * island.len();

    if (island.any(|&(_, _, c)| contact_equation::needs_first_order_resolution(c, self.depth_limit)))
    {
      self.resize_buffers(num_equations);

      // FIXME: move all those inside of the solver to reuse them (and avoid
      // allocations)
      let _0      = Zero::zero::<N>();
      let max_dim = Flatten::flat_size::<N, LV>() + Flatten::flat_size::<N, AV>();

      let mut ididx    = 0;
      let mut idJ      = 0;
      let mut idb      = 0;
      let mut idbounds = 0;

      for island.each |&(rb1, rb2, c)|
      {
        contact_equation::fill_first_order_contact_equation(
          dt,
          c,
          rb1, rb2,
          self.J, self.MJ, self.b, self.lambda, self.bounds, self.idx,
          &mut ididx, &mut idJ, &mut idb, &mut idbounds,
          self.depth_limit, self.corr_factor
        );
      }

      // FIXME: parametrize by the resolution algorithm?
      let MJLambda = pgs::projected_gauss_seidel_solve(
        self.J,
        self.MJ,
        self.b,
        self.lambda,
        self.bounds,
        self.idx,
        max_dim,
        num_equations,
        bodies.len(),
        self.num_first_order_iter
      );

      for bodies.each |&b|
      {
        if (b.proxy().index >= 0)
        {
          let offset = b.proxy().index as uint * max_dim;
          let dp     = Flatten::from_flattened::<N, LV>(MJLambda, offset).scalar_mul(&dt);
          let da     = Flatten::from_flattened::<N, AV>(
                         MJLambda,
                         offset + Flatten::flat_size::<N, LV>()
                       ).scalar_mul(&dt);

          let center = &b.local_to_world().translation();

          b.append(
            &rotation::rotate_wrt_point(&One::one::<M>(), &da, center)
            .translated(&dp)
          );
        }
      }
    }
  }

  fn second_order_solve(&mut self,
                        dt:     N,
                        island: &[(@mut RB, @mut RB, @mut C)],
                        bodies: &[@mut RB])
  {
    let equations_per_contacts = 1; // FIXME: add friction
    let num_equations          = equations_per_contacts * island.len();

    self.resize_buffers(num_equations);

    // FIXME: move all those inside of the solver to reuse them (and avoid
    // allocations)
    let _0      = Zero::zero::<N>();
    let max_dim = Flatten::flat_size::<N, LV>() + Flatten::flat_size::<N, AV>();

    let mut ididx    = 0;
    let mut idJ      = 0;
    let mut idb      = 0;
    let mut idbounds = 0;

    for island.each |&(rb1, rb2, c)|
    {
      contact_equation::fill_second_order_contact_equation(
        dt,
        c,
        rb1, rb2,
        self.J, self.MJ, self.b, self.lambda, self.bounds, self.idx,
        &mut ididx, &mut idJ, &mut idb, &mut idbounds,
        self.depth_limit, self.corr_factor, self.depth_eps
      );
    }

    // FIXME: parametrize by the resolution algorithm?
    let MJLambda = pgs::projected_gauss_seidel_solve(
      self.J,
      self.MJ,
      self.b,
      self.lambda,
      self.bounds,
      self.idx,
      max_dim,
      num_equations,
      bodies.len(),
      self.num_second_order_iter
      );

    for bodies.each |&b|
    {
      if (b.proxy().index >= 0)
      {
        let offset = b.proxy().index as uint * max_dim;
        let dlv    = Flatten::from_flattened::<N, LV>(MJLambda, offset);
        let dav    = Flatten::from_flattened::<N, AV>(
          MJLambda,
          offset + Flatten::flat_size::<N, LV>()
        );

        let curr_lin_vel = b.lin_vel();
        let curr_ang_vel = b.ang_vel();

        b.set_lin_vel(&(curr_lin_vel + dlv));
        b.set_ang_vel(&(curr_ang_vel + dav));
      }
    }

    // copy back the impulse buffer
    // FIXME: let mut i = 0u;

    // FIXME: for island.each |&(_, _, c)|
    // FIXME: {
    // FIXME:   c.set_impulse(self.lambda[i]);
    // FIXME:   i += 1;
    // FIXME: }
  }
}

impl<C:  ContactWithImpulse<LV, N>,
     LV: Flatten<N> + VectorSpace<N> + Cross<AV>  + Dot<N>,
     AV: Flatten<N> + VectorSpace<N> + Dot<N>,
     N:  DivisionRing + Orderable + Bounded + Copy,
     RB: Dynamic<N, LV, AV, II> + Translation<LV> + HasProxy<IndexProxy> +
         Transformable<M> + CanMove,
     II: RMul<AV>,
     M: Translation<LV> + Rotation<AV> + One>
    ConstraintSolver<N, RB, C> for
    AccumulatedImpulseSolver<N, C, LV, AV, RB, II, M>
{
  fn solve(&mut self,
           dt:     N,
           island: &[(@mut RB, @mut RB, @mut C)],
           bodies: &[@mut RB])
  {
    if (island.len() != 0)
    {
      let mut i = 0;

      for bodies.each |b|
      {
        if (!b.can_move())
        { b.proxy_mut().index = -1 }
        else
        {
          b.proxy_mut().index = i;
          i += 1;
        }
      }

      // FIXME: shuffle the constraints list
      self.second_order_solve(dt, island, bodies);
      self.first_order_solve(dt, island, bodies);
    }
  }
}
