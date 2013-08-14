use std::rand;
// use std::rand::RngUtil;
use std::num::{One, Orderable, Bounded};
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
use object::body::ToRigidBody;
use object::volumetric::InertiaTensor;
use resolution::constraint::velocity_constraint::VelocityConstraint;
use resolution::constraint::contact_equation;
use resolution::constraint::contact_equation::CorrectionParameters;
use resolution::solver::Solver;
use pgs = resolution::constraint::projected_gauss_seidel_solver;


pub struct AccumulatedImpulseSolver<N, LV, AV, M, II> {
    priv rng:                     rand::IsaacRng,
    priv correction:              CorrectionParameters<N>,
    priv num_first_order_iter:    uint,
    priv num_second_order_iter:   uint,
    priv restitution_constraints: ~[VelocityConstraint<LV, AV, N>],
    priv friction_constraints:    ~[VelocityConstraint<LV, AV, N>]
}

impl<LV: VectorSpace<N> + Cross<AV>  + Dot<N> + Basis + Dim +
         Clone + ToStr,
     AV: VectorSpace<N> + Dot<N> + ToStr + Clone,
     N:  DivisionRing + Orderable + Bounded + Signed + Clone +
         NumCast + ToStr,
     M:  Translation<LV> + Transform<LV> + Rotate<LV> + Translatable<LV, M> + Mul<M, M> +
         Rotation<AV> + One + Clone + Inv,
     II: Transform<AV> + Mul<II, II> + Inv + InertiaTensor<N, LV, M> + Clone>
AccumulatedImpulseSolver<N, LV, AV, M, II> {
    pub fn new(depth_limit:           N,
               corr_factor:           N,
               depth_eps:             N,
               rest_eps:              N,
               num_first_order_iter:  uint,
               num_second_order_iter: uint)
               -> AccumulatedImpulseSolver<N, LV, AV, M, II> {
        AccumulatedImpulseSolver {
            rng:                     rand::IsaacRng::new_seeded([42]),
            num_first_order_iter:    num_first_order_iter,
            num_second_order_iter:   num_second_order_iter,
            restitution_constraints: ~[],
            friction_constraints:    ~[],

            correction: CorrectionParameters {
                depth_limit: depth_limit,
                corr_factor: corr_factor,
                depth_eps:   depth_eps,
                rest_eps:    rest_eps
            }
        }
    }

    fn resize_buffers(&mut self, num_restitution_equations: uint, num_friction_equations: uint) {
        resize_buffer(&mut self.restitution_constraints,
                      num_restitution_equations,
                      VelocityConstraint::new());

        resize_buffer(&mut self.friction_constraints,
                      num_friction_equations,
                      VelocityConstraint::new());
    }

    fn first_order_solve(&mut self,
                         dt:          N,
                         constraints: &[Constraint<N, LV, AV, M, II>],
                         bodies:      &[@mut RigidBody<N, LV, AV, M, II>]) {
        let num_friction_equations    = 0;
        let num_restitution_equations = constraints.len();

        if constraints.iter().any(|&RBRB(_, _, ref c)| c.depth > self.correction.depth_limit) {
            self.resize_buffers(num_restitution_equations, num_friction_equations);

            for (i, &RBRB(rb1, rb2, ref c)) in constraints.iter().enumerate() {
                contact_equation::fill_first_order_contact_equation(
                    dt.clone(),
                    c,
                    rb1.to_rigid_body_or_fail(), rb2.to_rigid_body_or_fail(),
                    &mut self.restitution_constraints[i],
                    &self.correction);
            }

            // FIXME: parametrize by the resolution algorithm?
            let mut MJLambda = pgs::projected_gauss_seidel_solve(
                self.restitution_constraints,
                self.friction_constraints,
                bodies.len(),
                self.num_first_order_iter,
                true
                );

            for &b in bodies.iter() {
                let i = b.index();

                MJLambda[i].lv.scalar_mul_inplace(&dt);
                MJLambda[i].av.scalar_mul_inplace(&dt);

                let center = &b.center_of_mass().clone();

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
                          bodies:      &[@mut RigidBody<N, LV, AV, M, II>]) {
        let num_friction_equations    = (Dim::dim::<LV>() - 1) * constraints.len();
        let num_restitution_equations = constraints.len();

        self.resize_buffers(num_restitution_equations, num_friction_equations);

        for (i, &RBRB(rb1, rb2, ref c)) in constraints.iter().enumerate() {
            contact_equation::fill_second_order_contact_equation(
                dt.clone(),
                c,
                rb1.to_rigid_body_or_fail(), rb2.to_rigid_body_or_fail(),
                &mut self.restitution_constraints[i],
                i,
                self.friction_constraints,
                i * (Dim::dim::<LV>() - 1),
                &self.correction);
        }

        // FIXME: parametrize by the resolution algorithm?
        let MJLambda = pgs::projected_gauss_seidel_solve(
            self.restitution_constraints,
            self.friction_constraints,
            bodies.len(),
            self.num_second_order_iter,
            false);

        for &b in bodies.iter() {
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
     II: Transform<AV> + Mul<II, II> + Inv + Clone + InertiaTensor<N, LV, M>>
Solver<N, Constraint<N, LV, AV, M, II>> for
AccumulatedImpulseSolver<N, LV, AV, M, II> {
    fn solve(&mut self, dt: N, constraints: &[Constraint<N, LV, AV, M, II>]) {
        // XXX bodies index assignment is very uggly
        let mut bodies = ~[];

        if constraints.len() != 0 {
            // This is a two-passes assignation of index to the rigid bodies.
            // This is not very good, but seems to be the only way to do that without having a separate
            // list of all rigid bodies.
            for c in constraints.iter() {
                match *c {
                    RBRB(a, b, _) => {
                        a.set_index(-2);
                        b.set_index(-2)
                    }
                }
            }

            let mut id = 0;

            for c in constraints.iter() {
                match *c {
                    RBRB(a, b, _) => {
                        if a.index() == -2 {
                            if a.can_move() {
                                a.set_index(id);
                                bodies.push(a.to_rigid_body_or_fail());
                                id = id + 1;
                            }
                            else {
                                a.set_index(-1)
                            }
                        }
                        if b.index() == -2 {
                            if b.can_move() {
                                b.set_index(id);
                                bodies.push(b.to_rigid_body_or_fail());
                                id = id + 1;
                            }
                            else {
                                b.set_index(-1)
                            }
                        }
                    }
                }
            }

            self.second_order_solve(dt.clone(), constraints, bodies);
            self.first_order_solve(dt, constraints, bodies);
        }
    }
}

fn resize_buffer<A: Clone>(buff: &mut ~[A], size: uint, val: A) {
    if buff.len() < size {
        buff.grow_set(size - 1, &val, val.clone());
    }
    else {
        buff.truncate(size)
    }
}
