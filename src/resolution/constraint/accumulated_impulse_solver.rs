use std::num::Zero;
use std::rc::Rc;
use std::cell::RefCell;
// use std::rand::RngUtil;
use nalgebra::na::{Translation, Transformation, RotationWithTranslation};
use nalgebra::na;
use ncollide::math::{Scalar, Vector, Orientation, Matrix};
use detection::constraint::{Constraint, RBRB, BallInSocket, Fixed};
use object::RigidBody;
use resolution::constraint::velocity_constraint::VelocityConstraint;
use resolution::constraint::contact_equation;
use resolution::constraint::contact_equation::{CorrectionMode, CorrectionParameters};
use resolution::constraint::ball_in_socket_equation;
use resolution::constraint::fixed_equation;
use resolution::solver::Solver;
use pgs = resolution::constraint::projected_gauss_seidel_solver;
use resolution::constraint::projected_gauss_seidel_solver::Velocities;
use resolution::constraint::impulse_cache::ImpulseCache;


/// Constraint solver using the projected gauss seidel algorithm and warm-starting.
pub struct AccumulatedImpulseSolver {
    priv correction:              CorrectionParameters,
    priv cache:                   ImpulseCache,
    priv num_first_order_iter:    uint,
    priv num_second_order_iter:   uint,
    priv restitution_constraints: ~[VelocityConstraint],
    priv friction_constraints:    ~[VelocityConstraint],
    priv mjLambda:                ~[Velocities]
}

impl AccumulatedImpulseSolver {
    /// Creates a new `AccumulatedImpulseSolver`.
    pub fn new(step:                  Scalar,
               correction_mode:       CorrectionMode,
               joint_corr_factor:     Scalar,
               rest_eps:              Scalar,
               num_first_order_iter:  uint,
               num_second_order_iter: uint)
               -> AccumulatedImpulseSolver {
        AccumulatedImpulseSolver {
            num_first_order_iter:    num_first_order_iter,
            num_second_order_iter:   num_second_order_iter,
            restitution_constraints: ~[],
            friction_constraints:    ~[],
            mjLambda:                ~[],
            cache:                   ImpulseCache::new(step, na::dim::<Vector>()),

            correction: CorrectionParameters {
                corr_mode:  correction_mode,
                joint_corr: joint_corr_factor,
                rest_eps:   rest_eps
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

    fn do_solve(&mut self,
                dt:          Scalar,
                constraints: &[Constraint],
                joints:      &[uint],
                bodies:      &[Rc<RefCell<RigidBody>>]) {
        let num_friction_equations    = (na::dim::<Vector>() - 1) * self.cache.len();
        let num_restitution_equations = self.cache.len();
        let mut num_joint_equations = 0;

        for i in joints.iter() {
            match constraints[*i] {
                BallInSocket(_) => {
                    num_joint_equations = num_joint_equations + na::dim::<Vector>()
                },
                Fixed(_) => {
                    num_joint_equations = num_joint_equations + na::dim::<Vector>() + na::dim::<Orientation>()
                },
                RBRB(_, _, _) => { }
            }
        }

        self.resize_buffers(num_restitution_equations + num_joint_equations, num_friction_equations);

        let mut friction_offset = 0;

        for (i, (_, &(ci, imp))) in self.cache.hash().iter().enumerate() {
            match constraints[ci] {
                RBRB(ref rb1, ref rb2, ref c) => {
                    contact_equation::fill_second_order_equation(
                        dt.clone(),
                        c,
                        &rb1.borrow().borrow(), &rb2.borrow().borrow(),
                        &mut self.restitution_constraints[i],
                        i,
                        self.friction_constraints,
                        friction_offset,
                        self.cache.impulsions_at(imp),
                        &self.correction);
                },
                _ => { }
            }

            friction_offset = friction_offset + na::dim::<Vector>() - 1;
        }

        let mut joint_offset = num_restitution_equations;
        for i in joints.iter() {
            match constraints[*i] {
                BallInSocket(ref bis) => {
                    let bbis = bis.borrow().borrow();
                    ball_in_socket_equation::fill_second_order_equation(
                        dt.clone(),
                        bbis.get(),
                        self.restitution_constraints.mut_slice_from(joint_offset), // XXX
                        &self.correction
                    );

                    joint_offset = joint_offset + na::dim::<Vector>();
                },
                Fixed(ref f) => {
                    let bf = f.borrow().borrow();
                    fixed_equation::fill_second_order_equation(
                        dt.clone(),
                        bf.get(),
                        self.restitution_constraints.mut_slice_from(joint_offset), // XXX
                        &self.correction
                    );

                    joint_offset = joint_offset + na::dim::<Vector>() + na::dim::<Orientation>();
                },
                RBRB(_, _, _) => { }
            }
        }

        resize_buffer(&mut self.mjLambda, bodies.len(), Velocities::new());

        // FIXME: parametrize by the resolution algorithm?
        pgs::projected_gauss_seidel_solve(
            self.restitution_constraints,
            self.friction_constraints,
            self.mjLambda,
            bodies.len(),
            self.num_second_order_iter,
            false);

        // FIXME: this is _so_ ugly!
        self.resize_buffers(num_restitution_equations, num_friction_equations);

        for b in bodies.iter() {
            let mut bb = b.borrow().borrow_mut();
            let     rb = bb.get();
            let     i  = rb.index();

            let curr_lin_vel = rb.lin_vel();
            let curr_ang_vel = rb.ang_vel();

            rb.set_lin_vel(curr_lin_vel + self.mjLambda[i].lv);
            rb.set_ang_vel(curr_ang_vel + self.mjLambda[i].av);
        }

        for (i, dv) in self.restitution_constraints.iter().enumerate() {
            let imps = self.cache.push_impulsions();
            imps[0]  = dv.impulse * na::cast(0.85);

            for j in range(0u, na::dim::<Vector>() - 1) {
                let fc = &self.friction_constraints[i * (na::dim::<Vector>() - 1) + j];
                imps[1 + j] = fc.impulse * na::cast(0.85);
            }
        }

        let offset = self.cache.reserved_impulse_offset();
        for (i, (_, kv)) in self.cache.hash_mut().mut_iter().enumerate() {
            *kv = (kv.val0(), offset + i * na::dim::<Vector>());
        }

        /*
         * first order resolution
         */
        let needs_correction = !self.correction.corr_mode.pos_corr_factor().is_zero() &&
            constraints.iter().any(|constraint| {
            match *constraint {
                RBRB(_, _, ref c) => c.depth >= self.correction.corr_mode.min_depth_for_pos_corr(),
                _ => false // no first order resolution for joints
            }
        });

        if needs_correction {
            self.resize_buffers(num_restitution_equations, num_friction_equations);

            for (i, (_, &(ci, _))) in self.cache.hash().iter().enumerate() {
                match constraints[ci] {
                    RBRB(_, _, ref c) => {
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
                self.restitution_constraints,
                [],
                self.mjLambda,
                bodies.len(),
                self.num_first_order_iter,
                true);

            for b in bodies.iter() {
                let mut bb = b.borrow().borrow_mut();
                let rb     = bb.get();
                let i      = rb.index();

                let translation = self.mjLambda[i].lv * dt;
                let rotation    = self.mjLambda[i].av * dt;

                let center = &rb.center_of_mass().clone();

                let mut delta: Matrix = na::one();
                delta.append_rotation_wrt_point(&rotation, center);
                delta.append_translation(&translation);

                rb.append_transformation(&delta);
            }
        }
    }
}

impl Solver<Constraint> for AccumulatedImpulseSolver {
    fn solve(&mut self, dt: Scalar, constraints: &[Constraint]) {
        // FIXME: bodies index assignment is very ugly
        let mut bodies = ~[];

        if constraints.len() != 0 {
            /*
             * Associate the constraints with the cached impulse.
             */
            for (i, cstr) in constraints.iter().enumerate() {
                match *cstr {
                    RBRB(ref a, ref b, ref c) => {
                        self.cache.insert(i,
                                          a.borrow() as *RefCell<RigidBody> as uint,
                                          b.borrow() as *RefCell<RigidBody> as uint,
                                          (c.world1 + c.world2) / na::cast::<f32, Scalar>(2.0));
                    },
                    BallInSocket(_) => {
                        // XXX: cache for ball in socket?
                    },
                    Fixed(_) => {
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
                    RBRB(ref a, ref b, _) => {
                        let mut ba = a.borrow().borrow_mut();
                        let mut bb = b.borrow().borrow_mut();
                        ba.get().set_index(-2);
                        bb.get().set_index(-2)
                    },
                    BallInSocket(ref bis) => {
                        let bbis = bis.borrow().borrow();
                        match bbis.get().anchor1().body {
                            Some(ref b) => {
                                let mut bb = b.borrow().borrow_mut();
                                bb.get().set_index(-2)
                            },
                            None    => { }
                        };

                        match bbis.get().anchor2().body {
                            Some(ref b) => {
                                let mut bb = b.borrow().borrow_mut();
                                bb.get().set_index(-2)
                            },
                            None    => { }
                        }
                    }
                    Fixed(ref f) => { // FIXME: code duplication from BallInSocket
                        let bf = f.borrow().borrow();
                        match bf.get().anchor1().body {
                            Some(ref b) => {
                                let mut bb = b.borrow().borrow_mut();
                                bb.get().set_index(-2)
                            },
                            None    => { }
                        };

                        match bf.get().anchor2().body {
                            Some(ref b) => {
                                let mut bb = b.borrow().borrow_mut();
                                bb.get().set_index(-2)
                            },
                            None    => { }
                        }
                    }
                }
            }

            let mut id = 0;

            fn set_body_index(a: &Rc<RefCell<RigidBody>>, bodies: &mut ~[Rc<RefCell<RigidBody>>], id: &mut int) {
                let mut ba = a.borrow().borrow_mut();
                if ba.get().index() == -2 {
                    if ba.get().can_move() {
                        ba.get().set_index(*id);
                        bodies.push(a.clone());
                        *id = *id + 1;
                    }
                    else {
                        ba.get().set_index(-1)
                    }
                }
            }

            // FIXME: avoid allocation
            let mut joints = ~[];
            for (i, c) in constraints.iter().enumerate() {
                match *c {
                    RBRB(ref a, ref b, _) => {
                        set_body_index(a, &mut bodies, &mut id);
                        set_body_index(b, &mut bodies, &mut id);
                    },
                    BallInSocket(ref bis) => {
                        joints.push(i);
                        let bbis = bis.borrow().borrow();
                        match bbis.get().anchor1().body {
                            Some(ref b) => set_body_index(b, &mut bodies, &mut id),
                            None        => { }
                        }

                        match bbis.get().anchor2().body {
                            Some(ref b) => set_body_index(b, &mut bodies, &mut id),
                            None        => { }
                        }
                    },
                    Fixed(ref f) => { // FIXME: code duplication from BallInSocket
                        joints.push(i);
                        let bf = f.borrow().borrow();
                        match bf.get().anchor1().body {
                            Some(ref b) => set_body_index(b, &mut bodies, &mut id),
                            None        => { }
                        }

                        match bf.get().anchor2().body {
                            Some(ref b) => set_body_index(b, &mut bodies, &mut id),
                            None        => { }
                        }
                    }
                }
            }

            self.do_solve(dt.clone(), constraints, joints, bodies);
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
