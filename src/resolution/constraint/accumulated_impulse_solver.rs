use std::rand;
// use std::rand::RngUtil;
use std::num::{Zero, One, Orderable, Bounded};
use nalgebra::traits::dim::Dim;
use nalgebra::traits::inv::Inv;
use nalgebra::traits::vector_space::VectorSpace;
use nalgebra::traits::division_ring::DivisionRing;
use nalgebra::traits::translation::{Translation, Translatable};
use nalgebra::traits::rotation;
use nalgebra::traits::rotation::{Rotate, Rotation};
use nalgebra::traits::transformation::{Transform, Transformation};
use nalgebra::traits::cross::Cross;
use nalgebra::traits::dot::Dot;
use nalgebra::traits::basis::Basis;
use detection::collision::bodies_bodies::{Constraint, RBRB};
use object::rigid_body::RigidBody;
use resolution::constraint::velocity_constraint::VelocityConstraint;
use resolution::constraint::contact_equation;
use resolution::constraint::contact_equation::CorrectionParameters;
use resolution::solver::Solver;
use pgs = resolution::constraint::projected_gauss_seidel_solver;


pub struct AccumulatedImpulseSolver<N, LV, AV, M, II>
{
    priv rng:                   rand::IsaacRng,
    priv correction:            CorrectionParameters<N>,
    priv num_first_order_iter:  uint,
    priv num_second_order_iter: uint,
    priv constraints:           ~[VelocityConstraint<LV, AV, N>]
}

impl<LV: VectorSpace<N> + Cross<AV>  + Dot<N> + Basis + Dim +
         Clone + ToStr,
     AV: VectorSpace<N> + Dot<N> + ToStr + Clone,
     N:  DivisionRing + Orderable + Bounded + Signed + Clone +
         NumCast + ToStr,
     M:  Translation<LV> + Transform<LV> + Rotate<LV> + Translatable<LV, M> + Mul<M, M> +
         Rotation<AV> + One + Clone + Inv,
     II: Transform<AV> + Mul<II, II> + Clone>
AccumulatedImpulseSolver<N, LV, AV, M, II>
{
    pub fn new(depth_limit:           N,
               corr_factor:           N,
               depth_eps:             N,
               rest_eps:              N,
               num_first_order_iter:  uint,
               num_second_order_iter: uint)
               -> AccumulatedImpulseSolver<N, LV, AV, M, II>
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
                         dt:          N,
                         constraints: &[Constraint<N, LV, AV, M, II>],
                         bodies:      &[@mut RigidBody<N, LV, AV, M, II>])
    {
        let equations_per_contact = 1;
        let num_equations         = equations_per_contact * constraints.len();

        if constraints.iter().any(|&RBRB(_, _, ref c)| c.depth > self.correction.depth_limit)
        {
            self.resize_buffers(num_equations);

            let _0 = Zero::zero::<N>();

            for (i, &RBRB(rb1, rb2, ref c)) in constraints.iter().enumerate()
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

            for &b in bodies.iter()
            {
                let i = b.index();

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

    fn second_order_solve(&mut self,
                          dt:          N,
                          constraints: &[Constraint<N, LV, AV, M, II>],
                          bodies:      &[@mut RigidBody<N, LV, AV, M, II>])
    {
        let equations_per_contact = Dim::dim::<LV>(); // normal + friction
        let num_equations         = equations_per_contact * constraints.len();

        self.resize_buffers(num_equations);

        let _0      = Zero::zero::<N>();

        for (i, &RBRB(rb1, rb2, ref c)) in constraints.iter().enumerate()
        {
            contact_equation::fill_second_order_contact_equation(
                dt.clone(),
                c,
                rb1, rb2,
                self.constraints,
                i * equations_per_contact,
                &self.correction);
        }

        // FIXME: parametrize by the resolution algorithm?
        let MJLambda = pgs::projected_gauss_seidel_solve(
            self.constraints,
            bodies.len(),
            self.num_second_order_iter,
            false
            );

        for &b in bodies.iter()
        {
            let i = b.index();

            let curr_lin_vel = b.lin_vel();
            let curr_ang_vel = b.ang_vel();

            b.set_lin_vel(curr_lin_vel + MJLambda[i].lv);
            b.set_ang_vel(curr_ang_vel + MJLambda[i].av);
        }

        // FIXME: copy back the impulse cache
    }
}

impl<LV: VectorSpace<N> + Cross<AV>  + Dot<N> + Basis + Dim +
         Clone + ToStr,
     AV: VectorSpace<N> + Dot<N> + ToStr + Clone,
     N:  DivisionRing + Orderable + Bounded + Signed + Clone +
         NumCast + ToStr,
     M:  Translation<LV> + Transform<LV> + Rotate<LV> + Translatable<LV, M> + Mul<M, M> +
         Rotation<AV> + One + Clone + Inv,
     II: Transform<AV> + Mul<II, II> + Clone>
Solver<N, Constraint<N, LV, AV, M, II>> for
AccumulatedImpulseSolver<N, LV, AV, M, II>
{
    fn solve(&mut self, dt: N, constraints: &[Constraint<N, LV, AV, M, II>])
    {
        // XXX bodies index assignment is very uggly
        let mut bodies = ~[];

        if constraints.len() != 0
        {
            // This is a two-passes assignation of index to the rigid bodies.
            // This is not very good, but seems to be the only way to do that without having a separate
            // list of all rigid bodies.
            for c in constraints.iter()
            {
                match *c
                {
                    RBRB(a, b, _) => {
                        a.set_index(-2);
                        b.set_index(-2)
                    }
                }
            }

            let mut id = 0;

            for c in constraints.iter()
            {
                match *c
                {
                    RBRB(a, b, _) => {
                        if a.index() == -2
                        {
                            if a.can_move()
                            {
                                a.set_index(id);
                                bodies.push(a);
                                id = id + 1;
                            }
                            else
                            { a.set_index(-1) }
                        }
                        if b.index() == -2
                        {
                            if b.can_move()
                            {
                                b.set_index(id);
                                bodies.push(b);
                                id = id + 1;
                            }
                            else
                            { b.set_index(-1) }
                        }
                    }
                }
            }

            self.second_order_solve(dt.clone(), constraints, bodies);
            self.first_order_solve(dt, constraints, bodies);
        }
    }
}
