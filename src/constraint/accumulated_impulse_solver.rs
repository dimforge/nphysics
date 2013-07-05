use std::uint;
use std::rand;
// use std::rand::RngUtil;
use std::num::{Zero, One, Orderable, Bounded};
use nalgebra::traits::dim::Dim;
use nalgebra::traits::vector_space::VectorSpace;
use nalgebra::traits::division_ring::DivisionRing;
use nalgebra::traits::iterable::{Iterable, FromAnyIterator};
use nalgebra::traits::translation::{Translation, Translatable};
use nalgebra::traits::rotation;
use nalgebra::traits::rotation::Rotation;
use nalgebra::traits::transformation::{Transform, Transformation};
use nalgebra::traits::cross::Cross;
use nalgebra::traits::dot::Dot;
use nalgebra::traits::basis::Basis;
use body::material::Material;
use body::dynamic::Dynamic;
use body::can_move::CanMove;
use constraint::index_proxy::HasIndexProxy;
use constraint::contact_with_impulses::ContactWithImpulses;
use constraint::contact_equation;
use constraint::contact_equation::{CorrectionParameters, ConstraintsSystemParameters};
use constraint::constraint_solver::ConstraintSolver;
use pgs = constraint::projected_gauss_seidel_solver;


pub struct AccumulatedImpulseSolver<N, C, LV, AV, RB, II, M>
{
  priv rng:        rand::IsaacRng,
  priv correction: CorrectionParameters<N>,
  priv system:     ConstraintsSystemParameters<N>
}

impl<C:  ContactWithImpulses<LV, N>,
     LV: Iterable<N> + FromAnyIterator<N> + VectorSpace<N> + Cross<AV> +
         Dot<N> + Dim + Basis + Clone,
     AV: Iterable<N> + FromAnyIterator<N> + VectorSpace<N> + Dot<N> + Dim,
     N:  DivisionRing + Orderable + Bounded + Clone + ToStr + Copy, // FIXME: need Copy for grow_set
     RB: Dynamic<N, LV, AV, II> + Translation<LV> + HasIndexProxy + CanMove +
         Transformation<M> + Material<N>,
     II: Transform<AV>,
     M: Translation<LV> + Translatable<LV, M> + Rotation<AV> + One>
    AccumulatedImpulseSolver<N, C, LV, AV, RB, II, M>
{
  pub fn new(depth_limit:           N,
             corr_factor:           N,
             depth_eps:             N,
             rest_eps:              N,
             num_first_order_iter:  uint,
             num_second_order_iter: uint) ->
    AccumulatedImpulseSolver<N, C, LV, AV, RB, II, M>
  {
    AccumulatedImpulseSolver {
      rng:         rand::IsaacRng::new_seeded([42]),
      correction: CorrectionParameters {
        depth_limit: depth_limit,
        corr_factor: corr_factor,
        depth_eps:   depth_eps,
        rest_eps:    rest_eps
      },
      system: ConstraintsSystemParameters {
        dt: Zero::zero(),
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
  }

  fn resize_buffers(&mut self, num_equations: uint)
  {
    let _0      = Zero::zero::<N>();
    let max_dim = Dim::dim::<LV>() + Dim::dim::<AV>();

    self.system.J.grow_set(num_equations * 2 * max_dim, &_0, _0.clone());
    self.system.MJ.grow_set(num_equations * 2 * max_dim, &_0, _0.clone());
    self.system.b.grow_set(num_equations, &_0, _0.clone());
    self.system.lambda.grow_set(num_equations, &_0, _0.clone());
    self.system.bounds.grow_set(num_equations * 2, &_0, _0.clone());
    self.system.idx.grow_set(num_equations * 2, &0, 0);
  }

  fn first_order_solve(&mut self,
                       island: &[(@mut RB, @mut RB, @mut C)],
                       bodies: &[@mut RB])
  {
    let equations_per_contact = 1;
    let num_equations         = equations_per_contact * island.len();

    if island.iter().any_(|&(_, _, c)| contact_equation::needs_first_order_resolution(c, &self.correction.depth_limit))
    {
      self.resize_buffers(num_equations);

      let _0      = Zero::zero::<N>();
      let max_dim = Dim::dim::<LV>() + Dim::dim::<AV>();

      let mut ididx    = 0;
      let mut idJ      = 0;
      let mut idb      = 0;
      let mut idbounds = 0;

      for island.iter().advance |&(rb1, rb2, c)|
      {
        contact_equation::fill_first_order_contact_equation(
          c,
          rb1, rb2,
          &mut self.system,
          &mut ididx, &mut idJ, &mut idb, &mut idbounds,
          &self.correction
        );
      }

      // FIXME: parametrize by the resolution algorithm?
      let MJLambda = pgs::projected_gauss_seidel_solve(
        self.system.J,
        self.system.MJ,
        self.system.b,
        self.system.lambda,
        self.system.bounds,
        self.system.idx,
        max_dim,
        num_equations,
        bodies.len(),
        self.system.num_first_order_iter,
        true
      );

      let mut MJLambda_iter = MJLambda.iter();

      for bodies.iter().advance |&b|
      {
        if b.proxy().index >= 0
        {
          let mut dp : LV = FromAnyIterator::from_iterator(&mut MJLambda_iter);
          let mut da : AV = FromAnyIterator::from_iterator(&mut MJLambda_iter);

          dp.scalar_mul_inplace(&self.system.dt);
          da.scalar_mul_inplace(&self.system.dt);

          da = Zero::zero();

          let center = &b.translation();

          b.transform_by(
            &rotation::rotated_wrt_point(&One::one::<M>(), &da, center)
            .translated(&dp)
          );
        }
      }
    }
  }

  fn second_order_solve(&mut self,
                        island: &[(@mut RB, @mut RB, @mut C)],
                        bodies: &[@mut RB])
  {
    let equations_per_contact = Dim::dim::<LV>(); // normal + friction
    let num_equations         = equations_per_contact * island.len();

    self.resize_buffers(num_equations);

    let _0      = Zero::zero::<N>();
    let max_dim = Dim::dim::<LV>() + Dim::dim::<AV>();

    let mut ididx    = 0;
    let mut idJ      = 0;
    let mut idb      = 0;
    let mut idbounds = 0;

    for island.iter().advance |&(rb1, rb2, c)|
    {
      contact_equation::fill_second_order_contact_equation(
        c,
        rb1, rb2,
        &mut self.system,
        &mut ididx, &mut idJ, &mut idb, &mut idbounds,
        &self.correction
      );
    }

    // FIXME: parametrize by the resolution algorithm?
    let MJLambda = pgs::projected_gauss_seidel_solve(
      self.system.J,
      self.system.MJ,
      self.system.b,
      self.system.lambda,
      self.system.bounds,
      self.system.idx,
      max_dim,
      num_equations,
      bodies.len(),
      self.system.num_second_order_iter,
      false
    );

    let mut MJLambda_iter = MJLambda.iter();

    for bodies.iter().advance |&b|
    {
      if b.proxy().index >= 0
      {
        let dlv: LV = FromAnyIterator::from_iterator(&mut MJLambda_iter);
        let dav: AV = FromAnyIterator::from_iterator(&mut MJLambda_iter);

        let curr_lin_vel = b.lin_vel();
        let curr_ang_vel = b.ang_vel();

        b.set_lin_vel(&(curr_lin_vel + dlv));
        b.set_ang_vel(&(curr_ang_vel + dav));
      }
    }

    for island.iter().enumerate().advance |(i, &(_, _, c))|
    {
      for uint::iterate(0, equations_per_contact) |j|
      { c.impulses_mut()[j] = self.system.lambda[i * equations_per_contact + j].clone() };
    }
  }
}

impl<C:  ContactWithImpulses<LV, N>,
     LV: Iterable<N> + FromAnyIterator<N> + VectorSpace<N> + Cross<AV> +
         Basis + Dot<N> + Dim + Clone,
     AV: Iterable<N> + FromAnyIterator<N> + VectorSpace<N> + Dot<N> + Dim,
     N:  DivisionRing + Orderable + Bounded + Clone + ToStr + Copy,
     RB: Dynamic<N, LV, AV, II> + Translation<LV> + HasIndexProxy + CanMove +
         Transformation<M> + Material<N>,
     II: Transform<AV>,
     M: Translation<LV> + Translatable<LV, M> + Rotation<AV> + One>
    ConstraintSolver<N, RB, C> for
    AccumulatedImpulseSolver<N, C, LV, AV, RB, II, M>
{
  fn solve(&mut self,
           dt:     N,
           island: &mut [(@mut RB, @mut RB, @mut C)],
           bodies: &[@mut RB])
  {
    if island.len() != 0
    {
      let mut i = 0;

      for bodies.iter().advance |b|
      {
        if !b.can_move()
        { b.proxy_mut().index = -1 }
        else
        {
          b.proxy_mut().index = i;
          i = i + 1;
        }
      }

      self.system.dt = dt;
      // self.rng.shuffle_mut(island);
      self.second_order_solve(island, bodies);
      self.first_order_solve(island, bodies);
    }
  }
}
