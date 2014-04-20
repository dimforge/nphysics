use std::num::Bounded;
use nalgebra::na;
use ncollide::contact::Contact;
use ncollide::volumetric::InertiaTensor;
use ncollide::math::{Scalar, Vect, Orientation};
use resolution::constraint::velocity_constraint::VelocityConstraint;
use object::RigidBody;

/// The correction coefficient used by the constraint solver.
pub enum CorrectionMode {
    /// Penetration are solved by the penalty method.
    Velocity(Scalar),
    /// Penetration are solved by the penalty method together with a hard repositioning.
    VelocityAndPosition(Scalar, Scalar, Scalar),
    /// Penetration are solved by the penalty method together with a hard repositioning.
    ///
    /// The amount of velocity correction is bounded by threshold.
    VelocityAndPositionThresold(Scalar, Scalar, Scalar)
}

impl CorrectionMode {
    #[inline]
    /// The velocity correction coefficient.
    pub fn vel_corr_factor(&self) -> Scalar {
        match *self {
            Velocity(ref v)                          => v.clone(),
            VelocityAndPosition(ref v, _, _)         => v.clone(),
            VelocityAndPositionThresold(ref v, _, _) => v.clone()
        }
    }

    #[inline]
    /// The position correction coefficient.
    pub fn pos_corr_factor(&self) -> Scalar {
        match *self {
            VelocityAndPosition(_, ref p, _)         => p.clone(),
            VelocityAndPositionThresold(_, ref p, _) => p.clone(),
            Velocity(_)                              => na::zero()
        }
    }

    #[inline]
    /// The minimum penetration depth required to switch on the hard repositioning based method.
    pub fn min_depth_for_pos_corr(&self) -> Scalar {
        match *self {
            VelocityAndPosition(_, _, ref t)         => t.clone(),
            VelocityAndPositionThresold(_, _, ref t) => t.clone(),
            Velocity(_)                              => Bounded::max_value()
        }
    }

    #[inline]
    /// The max penetration depth the velocity correction will attempt to correct.
    pub fn max_depth_for_vel_corr(&self) -> Scalar {
        match *self {
            VelocityAndPosition(_, _, _)             => Bounded::max_value(),
            VelocityAndPositionThresold(_, _, ref t) => t.clone(),
            Velocity(_)                              => Bounded::max_value()
        }
    }
}

pub struct CorrectionParameters {
    pub corr_mode:       CorrectionMode,
    pub joint_corr:      Scalar,
    pub rest_eps:        Scalar
}

pub fn reinit_to_first_order_equation(dt:         Scalar,
                                      coll:       &Contact,
                                      constraint: &mut VelocityConstraint,
                                      correction: &CorrectionParameters) {
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

pub fn fill_second_order_equation(dt:           Scalar,
                                  coll:         &Contact,
                                  rb1:          &RigidBody,
                                  rb2:          &RigidBody,
                                  rconstraint:  &mut VelocityConstraint,
                                  idr:          uint,
                                  fconstraints: &mut [VelocityConstraint],
                                  idf:          uint,
                                  cache:        &[Scalar],
                                  correction:   &CorrectionParameters) {
    let restitution = rb1.restitution() * rb2.restitution();

    let center = (coll.world1 + coll.world2) * na::cast::<f32, Scalar>(0.5);

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

pub fn fill_constraint_geometry(normal:     Vect,
                                rot_axis1:  Orientation,
                                rot_axis2:  Orientation,
                                rb1:        &Option<&RigidBody>,
                                rb2:        &Option<&RigidBody>,
                                constraint: &mut VelocityConstraint) {
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

    let _1: Scalar = na::one();
    constraint.inv_projected_mass = _1 / constraint.inv_projected_mass;
}

fn fill_velocity_constraint(dt:              Scalar,
                            normal:          Vect,
                            center:          Vect,
                            restitution:     Scalar,
                            depth:           Scalar,
                            initial_impulse: Scalar,
                            lobound:         Scalar,
                            hibound:         Scalar,
                            rb1:             &RigidBody,
                            rb2:             &RigidBody,
                            constraint:      &mut VelocityConstraint,
                            correction:      &CorrectionParameters) {
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

pub fn relative_velocity(rb1:       &Option<&RigidBody>,
                         rb2:       &Option<&RigidBody>,
                         normal:    &Vect,
                         rot_axis1: &Orientation,
                         rot_axis2: &Orientation,
                         dt:        &Scalar)
                         -> Scalar {
    let mut dvel: Scalar = na::zero();

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
