use std::iter;
// use rand::RngUtil;
use alga::general::Real;
use na;
use math::{Vector, Orientation, Rotation, Translation, Isometry};
use detection::constraint::Constraint;
use detection::joint::Joint;
use world::RigidBodyStorage;
use resolution::constraint::velocity_constraint::VelocityConstraint;
use resolution::constraint::contact_equation;
use resolution::constraint::contact_equation::{CorrectionMode, CorrectionParameters};
use resolution::constraint::ball_in_socket_equation;
use resolution::constraint::fixed_equation;
use resolution::solver::Solver;
use resolution::constraint::projected_gauss_seidel_solver as pgs;
use resolution::constraint::projected_gauss_seidel_solver::Velocities;
use resolution::constraint::impulse_cache::ImpulseCache;


/// Constraint solver using the projected gauss seidel algorithm and warm-starting.
pub struct AccumulatedImpulseSolver<N: Real> {
    correction:              CorrectionParameters<N>,
    cache:                   ImpulseCache<N>,
    num_first_order_iter:    usize,
    num_second_order_iter:   usize,
    restitution_constraints: Vec<VelocityConstraint<N>>,
    friction_constraints:    Vec<VelocityConstraint<N>>,
    mj_lambda:               Vec<Velocities<N>>
}

impl<N: Real> AccumulatedImpulseSolver<N> {
    /// Creates a new `AccumulatedImpulseSolver`.
    pub fn new(step:                  N,
               correction_mode:       CorrectionMode<N>,
               joint_corr_factor:     N,
               rest_eps:              N,
               num_first_order_iter:  usize,
               num_second_order_iter: usize)
               -> AccumulatedImpulseSolver<N> {
        AccumulatedImpulseSolver {
            num_first_order_iter:    num_first_order_iter,
            num_second_order_iter:   num_second_order_iter,
            restitution_constraints: Vec::new(),
            friction_constraints:    Vec::new(),
            mj_lambda:               Vec::new(),
            cache:                   ImpulseCache::new(step, na::dimension::<Vector<N>>()),

            correction: CorrectionParameters {
                corr_mode:  correction_mode,
                joint_corr: joint_corr_factor,
                rest_eps:   rest_eps
            }
        }
    }

    /// Gets the number of iteration done by the penetration depth correction solver.
    #[inline]
    pub fn num_first_order_iter(&self) -> usize {
        self.num_first_order_iter
    }

    /// Sets the number of iteration done by the penetration depth correction solver.
    #[inline]
    pub fn set_num_first_order_iter(&mut self, num: usize) {
        self.num_first_order_iter = num
    }

    /// Gets the number of iteration done by the velocity constraint solver.
    #[inline]
    pub fn num_second_order_iter(&self) -> usize {
        self.num_second_order_iter
    }

    /// Sets the number of iteration done by the velocity constraint solver.
    #[inline]
    pub fn set_num_second_order_iter(&mut self, num: usize) {
        self.num_second_order_iter = num
    }

    fn resize_buffers(&mut self, num_restitution_equations: usize, num_friction_equations: usize) {
        resize_buffer(&mut self.restitution_constraints,
                      num_restitution_equations,
                      VelocityConstraint::new());

        resize_buffer(&mut self.friction_constraints,
                      num_friction_equations,
                      VelocityConstraint::new());
    }

    fn do_solve(&mut self,
                dt:          N,
                constraints: &[Constraint<N>],
                constrained_bodies: &[usize],
                bodies:      &mut RigidBodyStorage<N>) {
        let num_friction_equations    = (na::dimension::<Vector<N>>() - 1) * self.cache.len();
        let num_restitution_equations = self.cache.len();
        let mut num_joint_equations = 0;

        // TODO: maybe cache joint indexes or ask for it in argument
        for constraint in constraints.iter() {
            match *constraint {
                Constraint::BallInSocket(_) => {
                    num_joint_equations = num_joint_equations + na::dimension::<Vector<N>>()
                },
                Constraint::Fixed(_) => {
                    num_joint_equations = num_joint_equations +
                                          na::dimension::<Vector<N>>() +
                                          na::dimension::<Orientation<N>>()
                },
                Constraint::RBRB(_, _, _) => { }
            }
        }

        self.resize_buffers(num_restitution_equations + num_joint_equations, num_friction_equations);

        let mut friction_offset = 0;

        for (i, (_, &(ci, imp))) in self.cache.hash().iter().enumerate() {
            match constraints[ci] {
                Constraint::RBRB(rb1_uid, rb2_uid, ref c) => {
                    contact_equation::fill_second_order_equation(
                        dt.clone(),
                        c,
                        &bodies[rb1_uid], &bodies[rb2_uid],
                        &mut self.restitution_constraints[i],
                        i,
                        &mut self.friction_constraints[..],
                        friction_offset,
                        self.cache.impulsions_at(imp),
                        &self.correction);
                },
                _ => { }
            }

            friction_offset = friction_offset + na::dimension::<Vector<N>>() - 1;
        }

        let mut joint_offset = num_restitution_equations;
        // TODO: maybe cache joint indexes
        for constraint in constraints.iter() {
            let nconstraints = self.restitution_constraints.len();
            match *constraint {
                Constraint::BallInSocket(ref bis) => {
                    ball_in_socket_equation::fill_second_order_equation(
                        dt.clone(),
                        bis,
                        &mut self.restitution_constraints[joint_offset .. nconstraints], // XXX
                        &self.correction,
                        bodies,
                    );

                    joint_offset = joint_offset + na::dimension::<Vector<N>>();
                },
                Constraint::Fixed(ref f) => {
                    fixed_equation::fill_second_order_equation(
                        dt.clone(),
                        f,
                        &mut self.restitution_constraints[joint_offset .. nconstraints], // XXX
                        &self.correction,
                        bodies
                    );

                    joint_offset = joint_offset + na::dimension::<Vector<N>>() + na::dimension::<Orientation<N>>();
                },
                Constraint::RBRB(_, _, _) => { }
            }
        }

        resize_buffer(&mut self.mj_lambda, constrained_bodies.len(), Velocities::new());

        // FIXME: parametrize by the resolution algorithm?
        pgs::projected_gauss_seidel_solve(
            &mut self.restitution_constraints[..],
            &mut self.friction_constraints[..],
            &mut self.mj_lambda[..],
            constrained_bodies.len(),
            self.num_second_order_iter,
            false);

        // FIXME: this is _so_ ugly!
        self.resize_buffers(num_restitution_equations, num_friction_equations);

        for &uid in constrained_bodies.iter() {
            let rb = &mut bodies[uid];
            let i      = rb.index();

            let curr_lin_vel = rb.lin_vel();
            let curr_ang_vel = rb.ang_vel();

            rb.set_lin_vel_internal(curr_lin_vel + self.mj_lambda[i as usize].lv);
            rb.set_ang_vel_internal(curr_ang_vel + self.mj_lambda[i as usize].av);
        }

        for (i, dv) in self.restitution_constraints.iter().enumerate() {
            let imps = self.cache.push_impulsions();
            imps[0]  = dv.impulse * na::convert::<f64, N>(0.85f64);

            for j in 0usize .. na::dimension::<Vector<N>>() - 1 {
                let fc = &self.friction_constraints[i * (na::dimension::<Vector<N>>() - 1) + j];
                imps[1 + j] = fc.impulse * na::convert::<f64, N>(0.85f64);
            }
        }

        let offset = self.cache.reserved_impulse_offset();
        for (i, (_, kv)) in self.cache.hash_mut().iter_mut().enumerate() {
            *kv = (kv.0, offset + i * na::dimension::<Vector<N>>());
        }

        /*
         * first order resolution
         */
        let needs_correction = !self.correction.corr_mode.pos_corr_factor().is_zero() &&
            constraints.iter().any(|constraint| {
            match *constraint {
                Constraint::RBRB(_, _, ref c) =>
                    c.depth >= self.correction.corr_mode.min_depth_for_pos_corr(),
                _ => false // no first order resolution for joints
            }
        });

        if needs_correction {
            self.resize_buffers(num_restitution_equations, num_friction_equations);

            for (i, (_, &(ci, _))) in self.cache.hash().iter().enumerate() {
                match constraints[ci] {
                    Constraint::RBRB(_, _, ref c) => {
                        contact_equation::reinit_to_first_order_equation(
                            dt.clone(),
                            c,
                            &mut self.restitution_constraints[i],
                            &self.correction);
                    },
                    _ => { }
                }
            }

            // FIXME: parametrize by the resolution algorithm?
            pgs::projected_gauss_seidel_solve(
                &mut self.restitution_constraints[..],
                &mut [][..],
                &mut self.mj_lambda[..],
                constrained_bodies.len(),
                self.num_first_order_iter,
                true);

            for &uid in constrained_bodies.iter() {
                let rb = &mut bodies[uid];
                let i      = rb.index();

                let translation = Translation::from_vector(self.mj_lambda[i as usize].lv * dt);
                let rotation    = Rotation::from_scaled_axis(self.mj_lambda[i as usize].av * dt);

                let center = *rb.center_of_mass();

                let mut delta: Isometry<N> = na::one();
                delta.append_rotation_wrt_point_mut(&rotation, &center);
                delta.append_translation_mut(&translation);

                rb.append_transformation(&delta);
            }
        }
    }
}

impl<N: Real> Solver<N, Constraint<N>> for AccumulatedImpulseSolver<N> {
    fn solve(&mut self, dt: N, constraints: &[Constraint<N>], bodies: &mut RigidBodyStorage<N>) {
        // TODO: does this should be made in the function do_solve ?
        //       does the do_solve have sense to be called without that requirement ?
        if constraints.len() != 0 {
            /*
             * Associate the constraints with the cached impulse.
             */
            for (i, cstr) in constraints.iter().enumerate() {
                match *cstr {
                    Constraint::RBRB(a, b, ref c) => {
                        self.cache.insert(i, a, b, na::center(&c.world1, &c.world2));
                    },
                    Constraint::BallInSocket(_) => {
                        // XXX: cache for ball in socket?
                    },
                    Constraint::Fixed(_) => {
                        // XXX: cache for fixed?
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
                    Constraint::RBRB(a, b, _) => {
                        bodies[a].set_index(-2);
                        bodies[b].set_index(-2)
                    },
                    Constraint::BallInSocket(ref bis) => {
                        if let Some(b) = bis.anchor1().body {
                            bodies[b].set_index(-2)
                        }
                        if let Some(b) = bis.anchor2().body {
                            bodies[b].set_index(-2)
                        }
                    }
                    Constraint::Fixed(ref f) => { // FIXME: code duplication from BallInSocket
                        if let Some(b) = f.anchor1().body {
                            bodies[b].set_index(-2)
                        }
                        if let Some(b) = f.anchor2().body {
                            bodies[b].set_index(-2)
                        }
                    }
                }
            }

            // FIXME: bodies index assignment is very ugly
            let mut constrained_bodies = Vec::new();

            {
                let mut id = 0;
                let mut set_body_index = |uid: usize| {
                    let b = &mut bodies[uid];
                    if b.index() == -2 {
                        if b.can_move() {
                            b.set_index(id);
                            constrained_bodies.push(b.uid());
                            id += 1;
                        }
                        else {
                            b.set_index(-1)
                        }
                    }
                };

                // FIXME: avoid allocation
                for c in constraints.iter() {
                    match *c {
                        Constraint::RBRB(a, b, _) => {
                            set_body_index(a);
                            set_body_index(b);
                        },
                        Constraint::BallInSocket(ref bis) => {
                            if let Some(b) = bis.anchor1().body {
                                set_body_index(b);
                            }
                            if let Some(b) = bis.anchor2().body {
                                set_body_index(b);
                            }
                        },
                        Constraint::Fixed(ref f) => { // FIXME: code duplication from BallInSocket
                            if let Some(b) = f.anchor1().body {
                                set_body_index(b);
                            }
                            if let Some(b) = f.anchor2().body {
                                set_body_index(b);
                            }
                        }
                    }
                }
            }

            self.do_solve(dt.clone(), constraints, &*constrained_bodies, bodies);
            self.cache.swap();
        }
    }
}

fn resize_buffer<A: Clone>(buff: &mut Vec<A>, size: usize, val: A) {
    if buff.len() < size {
        let diff = size - buff.len();
        buff.extend(iter::repeat(val).take(diff));
    }
    else {
        buff.truncate(size)
    }
}
