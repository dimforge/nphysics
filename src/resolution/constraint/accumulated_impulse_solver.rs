use std::iter;
// use rand::RngUtil;
use alga::general::Real;
use na;
use math::{Vector, Orientation, Rotation, Translation, Isometry};
use detection::constraint::Constraint;
use detection::joint::Joint;
use object::RigidBody;
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
                joints:      &[usize],
                bodies:      &[::Rc<RigidBody<N>>]) {
        let num_friction_equations    = (na::dimension::<Vector<N>>() - 1) * self.cache.len();
        let num_restitution_equations = self.cache.len();
        let mut num_joint_equations = 0;

        for i in joints.iter() {
            match constraints[*i] {
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
                Constraint::RBRB(ref rb1, ref rb2, ref c) => {
                    contact_equation::fill_second_order_equation(
                        dt.clone(),
                        c,
                        &*rb1.borrow(), &*rb2.borrow(),
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
        for i in joints.iter() {
            let nconstraints = self.restitution_constraints.len();
            match constraints[*i] {
                Constraint::BallInSocket(ref bis) => {
                    ball_in_socket_equation::fill_second_order_equation(
                        dt.clone(),
                        &*bis.borrow(),
                        &mut self.restitution_constraints[joint_offset .. nconstraints], // XXX
                        &self.correction
                    );

                    joint_offset = joint_offset + na::dimension::<Vector<N>>();
                },
                Constraint::Fixed(ref f) => {
                    fixed_equation::fill_second_order_equation(
                        dt.clone(),
                        &*f.borrow(),
                        &mut self.restitution_constraints[joint_offset .. nconstraints], // XXX
                        &self.correction
                    );

                    joint_offset = joint_offset + na::dimension::<Vector<N>>() + na::dimension::<Orientation<N>>();
                },
                Constraint::RBRB(_, _, _) => { }
            }
        }

        resize_buffer(&mut self.mj_lambda, bodies.len(), Velocities::new());

        // FIXME: parametrize by the resolution algorithm?
        pgs::projected_gauss_seidel_solve(
            &mut self.restitution_constraints[..],
            &mut self.friction_constraints[..],
            &mut self.mj_lambda[..],
            bodies.len(),
            self.num_second_order_iter,
            false);

        // FIXME: this is _so_ ugly!
        self.resize_buffers(num_restitution_equations, num_friction_equations);

        for b in bodies.iter() {
            let mut rb = b.borrow_mut();
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
                bodies.len(),
                self.num_first_order_iter,
                true);

            for b in bodies.iter() {
                let mut rb = b.borrow_mut();
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
    fn solve(&mut self, dt: N, constraints: &[Constraint<N>]) {
        // FIXME: bodies index assignment is very ugly
        let mut bodies = Vec::new();

        if constraints.len() != 0 {
            /*
             * Associate the constraints with the cached impulse.
             */
            for (i, cstr) in constraints.iter().enumerate() {
                match *cstr {
                    Constraint::RBRB(ref a, ref b, ref c) => {
                        self.cache.insert(i,
                                          a.ptr() as usize,
                                          b.ptr() as usize,
                                          na::center(&c.world1, &c.world2));
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
                    Constraint::RBRB(ref a, ref b, _) => {
                        a.borrow_mut().set_index(-2);
                        b.borrow_mut().set_index(-2)
                    },
                    Constraint::BallInSocket(ref bis) => {
                        let bbis = bis.borrow();
                        match bbis.anchor1().body {
                            Some(ref b) => {
                                b.borrow_mut().set_index(-2)
                            },
                            None    => { }
                        };

                        match bbis.anchor2().body {
                            Some(ref b) => {
                                b.borrow_mut().set_index(-2)
                            },
                            None    => { }
                        }
                    }
                    Constraint::Fixed(ref f) => { // FIXME: code duplication from BallInSocket
                        let bf = f.borrow();
                        match bf.anchor1().body {
                            Some(ref b) => {
                                b.borrow_mut().set_index(-2)
                            },
                            None    => { }
                        };

                        match bf.anchor2().body {
                            Some(ref b) => {
                                b.borrow_mut().set_index(-2)
                            },
                            None    => { }
                        }
                    }
                }
            }

            let mut id = 0;

            fn set_body_index<N: Real>(a:      &::Rc<RigidBody<N>>,
                                       bodies: &mut Vec<::Rc<RigidBody<N>>>,
                                       id:     &mut isize) {
                let mut ba = a.borrow_mut();
                if ba.index() == -2 {
                    if ba.can_move() {
                        ba.set_index(*id);
                        bodies.push(a.clone());
                        *id = *id + 1;
                    }
                    else {
                        ba.set_index(-1)
                    }
                }
            }

            // FIXME: avoid allocation
            let mut joints = Vec::new();
            for (i, c) in constraints.iter().enumerate() {
                match *c {
                    Constraint::RBRB(ref a, ref b, _) => {
                        set_body_index(a, &mut bodies, &mut id);
                        set_body_index(b, &mut bodies, &mut id);
                    },
                    Constraint::BallInSocket(ref bis) => {
                        joints.push(i);
                        let bbis = bis.borrow();
                        match bbis.anchor1().body {
                            Some(ref b) => set_body_index(b, &mut bodies, &mut id),
                            None        => { }
                        }

                        match bbis.anchor2().body {
                            Some(ref b) => set_body_index(b, &mut bodies, &mut id),
                            None        => { }
                        }
                    },
                    Constraint::Fixed(ref f) => { // FIXME: code duplication from BallInSocket
                        joints.push(i);
                        let bf = f.borrow();
                        match bf.anchor1().body {
                            Some(ref b) => set_body_index(b, &mut bodies, &mut id),
                            None        => { }
                        }

                        match bf.anchor2().body {
                            Some(ref b) => set_body_index(b, &mut bodies, &mut id),
                            None        => { }
                        }
                    }
                }
            }

            self.do_solve(dt.clone(), constraints, &joints[..], &bodies[..]);
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
