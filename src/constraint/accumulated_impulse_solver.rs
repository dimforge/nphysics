use std::uint;
use std::rand;
// use std::rand::RngUtil;
use std::num::{Zero, One, Orderable, Bounded};
use nalgebra::traits::dim::Dim;
use nalgebra::traits::vector_space::VectorSpace;
use nalgebra::traits::division_ring::DivisionRing;
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
use constraint::velocity_constraint::VelocityConstraint;
use constraint::index_proxy::HasIndexProxy;
use constraint::contact_with_impulses::ContactWithImpulses;
use constraint::contact_equation;
use constraint::contact_equation::CorrectionParameters;
use constraint::constraint_solver::ConstraintSolver;
use pgs = constraint::projected_gauss_seidel_solver;


pub struct AccumulatedImpulseSolver<N, C, LV, AV, RB, II, M>
{
  priv rng:                   rand::IsaacRng,
  priv correction:            CorrectionParameters<N>,
  priv num_first_order_iter:  uint,
  priv num_second_order_iter: uint,
  priv constraints:           ~[VelocityConstraint<LV, AV, N>]
}

impl<C:  ContactWithImpulses<LV, N> + ToStr,
     LV: VectorSpace<N> + Cross<AV> + Dot<N> + Dim + Basis + Copy + Clone + ToStr,
     AV: VectorSpace<N> + Dot<N> + Dim + Copy + ToStr,
     N:  DivisionRing + Orderable + Bounded + Signed + Clone + ToStr + Copy, // FIXME: need Copy for grow_set
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
      rng:                   rand::IsaacRng::new_seeded([42]),
      num_first_order_iter:  num_first_order_iter,
      num_second_order_iter: num_second_order_iter,
      constraints:           ~[],

      correction: CorrectionParameters {
        depth_limit: depth_limit,
        corr_factor: corr_factor,
        depth_eps:   depth_eps,
        rest_eps:    rest_eps
      }
    }
  }

  fn resize_buffers(&mut self, num_equations: uint)
  {
    if self.constraints.len() < num_equations
    {
      self.constraints.grow_set(num_equations - 1,
                                &VelocityConstraint::new(),
                                VelocityConstraint::new());
    }
    else
    { self.constraints.truncate(num_equations) }
  }

  fn first_order_solve(&mut self,
                       dt:     N,
                       island: &[(@mut RB, @mut RB, @mut C)],
                       bodies: &[@mut RB])
  {
    let equations_per_contact = 1;
    let num_equations         = equations_per_contact * island.len();

    if island.iter().any(|&(_, _, c)| contact_equation::needs_first_order_resolution(c, &self.correction.depth_limit))
    {
      self.resize_buffers(num_equations);

      let _0 = Zero::zero::<N>();

      for island.iter().enumerate().advance |(i, &(rb1, rb2, c))|
      {
        contact_equation::fill_first_order_contact_equation(
          dt.clone(),
          c,
          rb1, rb2,
          &mut self.constraints[i],
          &self.correction
        );
      }

      // FIXME: parametrize by the resolution algorithm?
      let mut MJLambda = pgs::projected_gauss_seidel_solve(
        self.constraints,
        bodies.len(),
        self.num_first_order_iter,
        true
      );

      for bodies.iter().enumerate().advance |(i, &b)|
      {
        if b.proxy().index >= 0
        {
          MJLambda[i].lv.scalar_mul_inplace(&dt);
          MJLambda[i].av.scalar_mul_inplace(&dt);

          let center = &b.translation();

          b.transform_by(
            &rotation::rotated_wrt_point(&One::one::<M>(), &MJLambda[i].av, center)
            .translated(&MJLambda[i].lv)
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
    let equations_per_contact = Dim::dim::<LV>(); // normal + friction
    let num_equations         = equations_per_contact * island.len();

    self.resize_buffers(num_equations);

    let _0      = Zero::zero::<N>();

    for island.iter().enumerate().advance |(i, &(rb1, rb2, c))|
    {
      contact_equation::fill_second_order_contact_equation(
        dt.clone(),
        c,
        rb1, rb2,
        self.constraints,
        i * equations_per_contact,
        &self.correction
      );
    }

    // FIXME: parametrize by the resolution algorithm?
    let MJLambda = pgs::projected_gauss_seidel_solve(
      self.constraints,
      bodies.len(),
      self.num_second_order_iter,
      false
    );

    for bodies.iter().enumerate().advance |(i, &b)|
    {
      if b.proxy().index >= 0
      {
        let curr_lin_vel = b.lin_vel();
        let curr_ang_vel = b.ang_vel();

        b.set_lin_vel(&(curr_lin_vel + MJLambda[i].lv));
        b.set_ang_vel(&(curr_ang_vel + MJLambda[i].av));
      }
    }

    for island.iter().enumerate().advance |(i, &(_, _, c))|
    {
      for uint::iterate(0, equations_per_contact) |j|
      { c.impulses_mut()[j] = self.constraints[i * equations_per_contact + j].impulse.clone() };
    }
  }
}

impl<C:  ContactWithImpulses<LV, N> + ToStr,
     LV: VectorSpace<N> + Cross<AV> + Basis + Dot<N> + Dim + Copy + Clone + ToStr,
     AV: VectorSpace<N> + Dot<N> + Copy + Dim + ToStr,
     N:  DivisionRing + Orderable + Bounded + Signed + Clone + ToStr + Copy,
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

      // self.rng.shuffle_mut(island);
      self.second_order_solve(dt.clone(), island, bodies);
      self.first_order_solve(dt, island, bodies);
    }
  }
}
