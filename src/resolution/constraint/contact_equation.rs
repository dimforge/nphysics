use num::Float;
use na::{self, Bounded};
use ncollide::math::Scalar;
use ncollide::geometry::Contact;
use volumetric::InertiaTensor;
use resolution::constraint::velocity_constraint::VelocityConstraint;
use object::RigidBody;
use math::{Point, Vector, Orientation};

/// The correction coefficient used by the constraint solver.
pub enum CorrectionMode<N: Scalar> {
    /// Penetration are solved by the penalty method.
    Velocity(N),
    /// Penetration are solved by the penalty method together with a hard repositioning.
    VelocityAndPosition(N, N, N),
    /// Penetration are solved by the penalty method together with a hard repositioning.
    ///
    /// The amount of velocity correction is bounded by threshold.
    VelocityAndPositionThresold(N, N, N)
}

impl<N: Scalar> CorrectionMode<N> {
    #[inline]
    /// The velocity correction coefficient.
    pub fn vel_corr_factor(&self) -> N {
        match *self {
            CorrectionMode::Velocity(ref v)                          => v.clone(),
            CorrectionMode::VelocityAndPosition(ref v, _, _)         => v.clone(),
            CorrectionMode::VelocityAndPositionThresold(ref v, _, _) => v.clone()
        }
    }

    #[inline]
    /// The position correction coefficient.
    pub fn pos_corr_factor(&self) -> N {
        match *self {
            CorrectionMode::VelocityAndPosition(_, ref p, _)         => p.clone(),
            CorrectionMode::VelocityAndPositionThresold(_, ref p, _) => p.clone(),
            CorrectionMode::Velocity(_)                              => na::zero()
        }
    }

    #[inline]
    /// The minimum penetration depth required to switch on the hard repositioning based method.
    pub fn min_depth_for_pos_corr(&self) -> N {
        match *self {
            CorrectionMode::VelocityAndPosition(_, _, ref t)         => t.clone(),
            CorrectionMode::VelocityAndPositionThresold(_, _, ref t) => t.clone(),
            CorrectionMode::Velocity(_)                              => Bounded::max_value()
        }
    }

    #[inline]
    /// The max penetration depth the velocity correction will attempt to correct.
    pub fn max_depth_for_vel_corr(&self) -> N {
        match *self {
            CorrectionMode::VelocityAndPosition(_, _, _)             => Bounded::max_value(),
            CorrectionMode::VelocityAndPositionThresold(_, _, ref t) => t.clone(),
            CorrectionMode::Velocity(_)                              => Bounded::max_value()
        }
    }
}

pub struct CorrectionParameters<N: Scalar> {
    pub corr_mode:       CorrectionMode<N>,
    pub joint_corr:      N,
    pub rest_eps:        N
}

pub fn reinit_to_first_order_equation<N: Scalar>(dt:         N,
                                                 coll:       &Contact<Point<N>>,
                                                 constraint: &mut VelocityConstraint<N>,
                                                 correction: &CorrectionParameters<N>) {
    /*
     * Fill b
     */
    if coll.depth >= correction.corr_mode.min_depth_for_pos_corr() {
        constraint.objective = correction.corr_mode.pos_corr_factor() * coll.depth.max(na::zero()) / dt;
    }
    else {
        constraint.objective = na::zero();
    }

    /*
     * Reset forces
     */
    constraint.impulse = na::zero();
}

pub fn fill_second_order_equation<N: Scalar>(dt:           N,
                                             coll:         &Contact<Point<N>>,
                                             rb1:          &RigidBody<N>,
                                             rb2:          &RigidBody<N>,
                                             rconstraint:  &mut VelocityConstraint<N>,
                                             idr:          usize,
                                             fconstraints: &mut [VelocityConstraint<N>],
                                             idf:          usize,
                                             cache:        &[N],
                                             correction:   &CorrectionParameters<N>) {
    let restitution = rb1.restitution() * rb2.restitution();

    let center = na::center(&coll.world1, &coll.world2);

    fill_velocity_constraint(dt.clone(),
                             coll.normal.clone(),
                             center.clone(),
                             restitution,
                             coll.depth.clone(),
                             cache[0].clone(), // coll.impulses[0].clone(),
                             na::zero(),
                             Bounded::max_value(),
                             rb1,
                             rb2,
                             rconstraint,
                             correction);


    let friction  = rb1.friction() * rb2.friction();
    // To bound the friction we use the last frame normal impulse.
    // That means we have to make a special case for the first time the contact appears.
    // In that case, we estimate the impulse by the derired normal correction.

    let mut i = 0;

    na::orthonormal_subspace_basis(&coll.normal, |friction_axis| {
        let constraint = &mut fconstraints[idf + i];

        fill_velocity_constraint(dt.clone(),
                                 friction_axis,
                                 center.clone(),
                                 na::zero(),
                                 na::zero(),
                                 cache[i + 1].clone(), // coll.impulses[i].clone(),
                                 na::zero(), // dont setup the limit now
                                 na::zero(), // dont setup the limit now
                                 rb1,
                                 rb2,
                                 constraint,
                                 correction);

        constraint.friction_coeff    = friction.clone();
        constraint.friction_limit_id = idr;
        i = i + 1;

        true
    })
}

pub fn fill_constraint_geometry<N: Scalar>(normal:     Vector<N>,
                                           rot_axis1:  Orientation<N>,
                                           rot_axis2:  Orientation<N>,
                                           rb1:        &Option<&RigidBody<N>>,
                                           rb2:        &Option<&RigidBody<N>>,
                                           constraint: &mut VelocityConstraint<N>) {
    constraint.normal             = normal;
    constraint.inv_projected_mass = na::zero();

    match *rb1 {
        Some(ref rb) => {
            // rotation axis
            constraint.weighted_normal1   = constraint.normal * rb.inv_mass();
            constraint.rot_axis1          = rot_axis1;

            constraint.weighted_rot_axis1 = rb.inv_inertia().apply(&constraint.rot_axis1);

            constraint.inv_projected_mass = constraint.inv_projected_mass +
                na::dot(&constraint.normal, &constraint.weighted_normal1) +
                na::dot(&constraint.rot_axis1, &constraint.weighted_rot_axis1);
        },
        None => { }
    }

    match *rb2 {
        Some(ref rb) => {
            // rotation axis
            constraint.weighted_normal2   = constraint.normal * rb.inv_mass();
            constraint.rot_axis2          = rot_axis2;

            constraint.weighted_rot_axis2 = rb.inv_inertia().apply(&constraint.rot_axis2);

            constraint.inv_projected_mass = constraint.inv_projected_mass +
                na::dot(&constraint.normal, &constraint.weighted_normal2) +
                na::dot(&constraint.rot_axis2, &constraint.weighted_rot_axis2);
        },
        None => { }
    }

    let _1: N = na::one();
    constraint.inv_projected_mass = _1 / constraint.inv_projected_mass;
}

fn fill_velocity_constraint<N: Scalar>(dt:              N,
                                       normal:          Vector<N>,
                                       center:          Point<N>,
                                       restitution:     N,
                                       depth:           N,
                                       initial_impulse: N,
                                       lobound:         N,
                                       hibound:         N,
                                       rb1:             &RigidBody<N>,
                                       rb2:             &RigidBody<N>,
                                       constraint:      &mut VelocityConstraint<N>,
                                       correction:      &CorrectionParameters<N>) {
    let rot_axis1 = na::cross(&(center - *rb1.center_of_mass()), &-normal);
    let rot_axis2 = na::cross(&(center - *rb2.center_of_mass()), &normal);

    let opt_rb1 = if rb1.can_move() { Some(rb1) } else { None };
    let opt_rb2 = if rb2.can_move() { Some(rb2) } else { None };
    fill_constraint_geometry(normal, rot_axis1, rot_axis2, &opt_rb1, &opt_rb2, constraint);

    /*
     * Fill indice
     */
    constraint.id1 = rb1.index();
    constraint.id2 = rb2.index();

    /*
     * correction amount
     */
    constraint.objective = relative_velocity(
        &opt_rb1,
        &opt_rb2,
        &constraint.normal,
        &constraint.rot_axis1,
        &constraint.rot_axis2,
        &dt);

    if constraint.objective < -correction.rest_eps {
        constraint.objective = constraint.objective + restitution * constraint.objective
    }

    constraint.objective = -constraint.objective;

    if depth < na::zero() {
        constraint.objective = constraint.objective + depth / dt
    }
    else if depth < correction.corr_mode.max_depth_for_vel_corr() {
        constraint.objective = constraint.objective + depth * correction.corr_mode.vel_corr_factor() / dt
    }

    // for warm-starting
    constraint.impulse = if depth < na::zero() { na::zero() } else { initial_impulse };

    /*
     * constraint bounds
     */
    constraint.lobound = lobound;
    constraint.hibound = hibound;
}

pub fn relative_velocity<N: Scalar>(rb1:       &Option<&RigidBody<N>>,
                                    rb2:       &Option<&RigidBody<N>>,
                                    normal:    &Vector<N>,
                                    rot_axis1: &Orientation<N>,
                                    rot_axis2: &Orientation<N>,
                                    dt:        &N)
                                    -> N {
    let mut dvel: N = na::zero();

    match *rb1 {
        Some(ref rb) => {
            dvel = dvel - na::dot(&(rb.lin_vel() + rb.lin_acc() * *dt), normal)
                        + na::dot(&(rb.ang_vel() + rb.ang_acc() * *dt), rot_axis1);
        },
        None => { }
    }

    match *rb2 {
        Some(ref rb) => {
            dvel = dvel + na::dot(&(rb.lin_vel() + rb.lin_acc() * *dt), normal)
                        + na::dot(&(rb.ang_vel() + rb.ang_acc() * *dt), rot_axis2);
        },
        None => { }
    }

    dvel
}
