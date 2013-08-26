use std::ptr;
// use std::rand::RngUtil;
use std::num::{One, Orderable, Bounded};
use nalgebra::traits::dim::Dim;
use nalgebra::traits::inv::Inv;
use nalgebra::traits::translation::Translation;
use nalgebra::traits::rotation;
use nalgebra::traits::rotation::{Rotate, Rotation};
use nalgebra::traits::transformation::{Transform, Transformation};
use nalgebra::traits::cross::Cross;
use nalgebra::traits::vector::{Vec, VecExt};
use detection::constraint::{Constraint, RBRB};
use object::rigid_body::RigidBody;
use object::body::ToRigidBody;
use object::volumetric::InertiaTensor;
use resolution::constraint::velocity_constraint::VelocityConstraint;
use resolution::constraint::contact_equation;
use resolution::constraint::contact_equation::{CorrectionMode, CorrectionParameters};
use resolution::solver::Solver;
use pgs = resolution::constraint::projected_gauss_seidel_solver;
use resolution::constraint::impulse_cache::ImpulseCache;


pub struct AccumulatedImpulseSolver<N, LV, AV, M, II> {
    priv correction:              CorrectionParameters<N>,
    priv cache:                   ImpulseCache<N, LV>,
    priv num_first_order_iter:    uint,
    priv num_second_order_iter:   uint,
    priv restitution_constraints: ~[VelocityConstraint<LV, AV, N>],
    priv friction_constraints:    ~[VelocityConstraint<LV, AV, N>]
}

impl<LV:  VecExt<N> + Cross<AV> + IterBytes + Clone + ToStr,
     AV:  Vec<N> + ToStr + Clone,
     N:   Num + Orderable + Bounded + Signed + Clone + NumCast + ToStr,
     M:   Translation<LV> + Transform<LV> + Rotate<LV> + Mul<M, M> +
          Rotation<AV> + One + Clone + Inv,
     II:  Transform<AV> + Mul<II, II> + Inv + InertiaTensor<N, LV, M> + Clone>
AccumulatedImpulseSolver<N, LV, AV, M, II> {
    pub fn new(step:                  N,
               correction_mode:       CorrectionMode<N>,
               rest_eps:              N,
               num_first_order_iter:  uint,
               num_second_order_iter: uint)
               -> AccumulatedImpulseSolver<N, LV, AV, M, II> {
        AccumulatedImpulseSolver {
            num_first_order_iter:    num_first_order_iter,
            num_second_order_iter:   num_second_order_iter,
            restitution_constraints: ~[],
            friction_constraints:    ~[],
            cache:                   ImpulseCache::new(step, Dim::dim::<LV>()),

            correction: CorrectionParameters {
                corr_mode:   correction_mode,
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

        let needs_correction = do constraints.iter().any |&RBRB(_, _, ref c)| {
            c.depth >= self.correction.corr_mode.min_depth_for_pos_corr()
        };

        if needs_correction {
            self.resize_buffers(num_restitution_equations, num_friction_equations);

            for (i, (_, &(ci, _))) in self.cache.hash().iter().enumerate() {
                match constraints[ci] {
                    RBRB(rb1, rb2, ref c) => {
                        contact_equation::fill_first_order_contact_equation(
                            dt.clone(),
                            c,
                            rb1.to_rigid_body_or_fail(), rb2.to_rigid_body_or_fail(),
                            &mut self.restitution_constraints[i],
                            &self.correction);
                    }
                }
            }

            // FIXME: parametrize by the resolution algorithm?
            let mut MJLambda = pgs::projected_gauss_seidel_solve(
                self.restitution_constraints,
                [],
                bodies.len(),
                self.num_first_order_iter,
                true);

            for &b in bodies.iter() {
                let i = b.index();

                MJLambda[i].lv = MJLambda[i].lv * dt;
                MJLambda[i].av = MJLambda[i].av * dt;

                let center = &b.center_of_mass().clone();

                b.transform_by(
                    &rotation::rotated_wrt_point(&One::one::<M>(), &MJLambda[i].av, center)
                    .translated(&MJLambda[i].lv));
            }
        }
    }

    fn second_order_solve(&mut self,
                          dt:          N,
                          constraints: &[Constraint<N, LV, AV, M, II>],
                          bodies:      &[@mut RigidBody<N, LV, AV, M, II>]) {
        let num_friction_equations    = (Dim::dim::<LV>() - 1) * self.cache.len();
        let num_restitution_equations = self.cache.len();

        self.resize_buffers(num_restitution_equations, num_friction_equations);

        for (i, (_, &(ci, imp))) in self.cache.hash().iter().enumerate() {
            match constraints[ci] {
                RBRB(rb1, rb2, ref c) => {
                    contact_equation::fill_second_order_contact_equation(
                        dt.clone(),
                        c,
                        rb1.to_rigid_body_or_fail(), rb2.to_rigid_body_or_fail(),
                        &mut self.restitution_constraints[i],
                        i,
                        self.friction_constraints,
                        i * (Dim::dim::<LV>() - 1),
                        self.cache.impulsions_at(imp),
                        &self.correction);
                }
            }
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

        for (i, dv) in self.restitution_constraints.iter().enumerate() {
            let imps = self.cache.push_impulsions();
            imps[0]  = dv.impulse * NumCast::from(0.85);

            for j in range(0u, Dim::dim::<LV>() - 1) {
                let fc = &self.friction_constraints[i * (Dim::dim::<LV>() - 1) + j];
                imps[1 + j] = fc.impulse * NumCast::from(0.85);
            }
        }

        let offset = self.cache.reserved_impulse_offset();
        for (i, (_, kv)) in self.cache.hash_mut().mut_iter().enumerate() {
            *kv = (kv.first(), offset + i * Dim::dim::<LV>());
        }
    }
}

impl<LV: VecExt<N> + Cross<AV> + IterBytes + Clone + ToStr,
     AV: Vec<N> + ToStr + Clone,
     N:  Num + Orderable + Bounded + Signed + Clone + NumCast + ToStr,
     M:  Translation<LV> + Transform<LV> + Rotate<LV> + Mul<M, M> + Rotation<AV> + One + Clone + Inv,
     II: Transform<AV> + Mul<II, II> + Inv + Clone + InertiaTensor<N, LV, M>>
Solver<N, Constraint<N, LV, AV, M, II>> for
AccumulatedImpulseSolver<N, LV, AV, M, II> {
    fn solve(&mut self, dt: N, constraints: &[Constraint<N, LV, AV, M, II>]) {
        // FIXME: bodies index assignment is very uggly
        let mut bodies = ~[];

        if constraints.len() != 0 {
            /*
             * Associate the constraints with the cached impulse.
             */
            for (i, cstr) in constraints.iter().enumerate() {
                match *cstr {
                    RBRB(a, b, ref c) => {
                        self.cache.insert(i,
                                          ptr::to_mut_unsafe_ptr(a) as uint,
                                          ptr::to_mut_unsafe_ptr(b) as uint,
                                          (c.world1 + c.world2) / NumCast::from(2.0));
                    }
                }
            }

            /*
             * Assign an index to each body.
             */
            // This is a two-passes assignation of index to the rigid bodies.
            // This is not very good, but is the only way to do that without having a separate list
            // of all rigid bodies.
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
            if !self.correction.corr_mode.pos_corr_factor().is_zero() {
                self.first_order_solve(dt, constraints, bodies)
            }
            self.cache.swap();
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
