use std::num::{One, Zero, Orderable, Bounded};
use nalgebra::traits::basis::Basis;
use nalgebra::traits::cross::Cross;
use nalgebra::traits::transformation::Transform;
use nalgebra::traits::rotation::Rotate;
use nalgebra::traits::vector::{Vec, VecExt};
use ncollide::contact::Contact;
use resolution::constraint::velocity_constraint::VelocityConstraint;
use object::rigid_body::RigidBody;
use object::volumetric::InertiaTensor;

pub enum CorrectionMode<N> {
    Velocity(N),
    VelocityAndPosition(N, N, N),
    VelocityAndPositionThresold(N, N, N)
}

impl<N: Zero + Bounded + Clone> CorrectionMode<N> {
    #[inline]
    pub fn vel_corr_factor(&self) -> N {
        match *self {
            Velocity(ref v)                          => v.clone(),
            VelocityAndPosition(ref v, _, _)         => v.clone(),
            VelocityAndPositionThresold(ref v, _, _) => v.clone()
        }
    }

    #[inline]
    pub fn pos_corr_factor(&self) -> N {
        match *self {
            VelocityAndPosition(_, ref p, _)         => p.clone(),
            VelocityAndPositionThresold(_, ref p, _) => p.clone(),
            Velocity(_)                              => Zero::zero()
        }
    }

    #[inline]
    pub fn min_depth_for_pos_corr(&self) -> N {
        match *self {
            VelocityAndPosition(_, _, ref t)         => t.clone(),
            VelocityAndPositionThresold(_, _, ref t) => t.clone(),
            Velocity(_)                              => Bounded::max_value()
        }
    }

    #[inline]
    pub fn max_depth_for_vel_corr(&self) -> N {
        match *self {
            VelocityAndPosition(_, _, _)             => Bounded::max_value(),
            VelocityAndPositionThresold(_, _, ref t) => t.clone(),
            Velocity(_)                              => Bounded::max_value()
        }
    }
}

pub struct CorrectionParameters<N> {
    corr_mode:       CorrectionMode<N>,
    rest_eps:        N
}

pub fn fill_first_order_contact_equation<LV: Vec<N> + Cross<AV> + Clone,
                                         AV: Vec<N>,
                                         N:  Num + Orderable + Bounded + NumCast + Clone + ToStr,
                                         M:  Transform<LV> + Rotate<LV> + One,
                                         II: Transform<AV> + Mul<II, II> + InertiaTensor<N, LV, M> + Clone>(
                                         dt:          N,
                                         coll:        &Contact<N, LV>,
                                         rb1:         &RigidBody<N, LV, AV, M, II>,
                                         rb2:         &RigidBody<N, LV, AV, M, II>,
                                         constraint:  &mut VelocityConstraint<LV, AV, N>,
                                         correction:  &CorrectionParameters<N>) {
    let center = (coll.world1 + coll.world2) / NumCast::from(2.0f64);

    fill_constraint_geometry(coll.normal.clone(), center, rb1, rb2, constraint);

    /*
     * Fill indice
     */
    constraint.id1 = rb1.index();
    constraint.id2 = rb2.index();

    /*
     * Fill b
     */
    if coll.depth >= correction.corr_mode.min_depth_for_pos_corr() {
        constraint.objective = correction.corr_mode.pos_corr_factor() * coll.depth.max(&Zero::zero()) / dt;
    }

    /*
     * Reset forces
     */
    constraint.impulse = Zero::zero();

    /*
     * Fill bounds
     */
    constraint.lobound = Zero::zero();
    constraint.hibound = Bounded::max_value();
}

// FIXME: note that removing the Clone constraint on N leads to an ICE
pub fn fill_second_order_contact_equation<LV: VecExt<N> + Cross<AV> + Clone,
                                          AV: Vec<N> + Clone,
                                          N:  Num + Orderable + Bounded + Signed + Clone +
                                              NumCast + ToStr,
                                          M:  Transform<LV> + Rotate<LV> + One,
                                          II: Transform<AV> + Mul<II, II> + InertiaTensor<N, LV, M> +
                                              Clone>(
                                          dt:           N,
                                          coll:         &Contact<N, LV>,
                                          rb1:          &RigidBody<N, LV, AV, M, II>,
                                          rb2:          &RigidBody<N, LV, AV, M, II>,
                                          rconstraint:  &mut VelocityConstraint<LV, AV, N>,
                                          idr:          uint,
                                          fconstraints: &mut [VelocityConstraint<LV, AV, N>],
                                          idf:          uint,
                                          cache:        &[N],
                                          correction:   &CorrectionParameters<N>) {
    let restitution = rb1.restitution() * rb2.restitution();

    let center = (coll.world1 + coll.world2) / NumCast::from(2.0f64);

    fill_velocity_constraint(dt.clone(),
                             coll.normal.clone(),
                             center.clone(),
                             restitution,
                             coll.depth.clone(),
                             cache[0].clone(), // coll.impulses[0].clone(),
                             Zero::zero(),
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

    do coll.normal.orthonormal_subspace_basis() |friction_axis| {
        let constraint = &mut fconstraints[idf + i];

        fill_velocity_constraint(dt.clone(),
                                 friction_axis,
                                 center.clone(),
                                 Zero::zero(),
                                 Zero::zero(),
                                 cache[i + 1].clone(), // coll.impulses[i].clone(),
                                 Zero::zero(), // dont setup the limit now
                                 Zero::zero(), // dont setup the limit now
                                 rb1,
                                 rb2,
                                 constraint,
                                 correction);

        constraint.friction_coeff    = friction.clone();
        constraint.friction_limit_id = idr;
        i = i + 1;

        true
    }
}

fn fill_constraint_geometry<LV: Vec<N> + Cross<AV> + Clone,
                            AV: Vec<N>,
                            N:  Num + Clone,
                            M:  Transform<LV> + Rotate<LV> + One,
                            II: Transform<AV> + Mul<II, II> + InertiaTensor<N, LV, M> + Clone>(
                            normal:     LV,
                            center:     LV,
                            rb1:        &RigidBody<N, LV, AV, M, II>,
                            rb2:        &RigidBody<N, LV, AV, M, II>,
                            constraint: &mut VelocityConstraint<LV, AV, N>) {
    constraint.normal             = normal;
    constraint.inv_projected_mass = Zero::zero();

    if rb1.can_move() {
        // rotation axis
        constraint.weighted_normal1   = constraint.normal * rb1.inv_mass();
        constraint.rot_axis1          = (center - *rb1.center_of_mass()).cross(&-constraint.normal);

        constraint.weighted_rot_axis1 = rb1.inv_inertia().transform(&constraint.rot_axis1);

        constraint.inv_projected_mass = constraint.inv_projected_mass +
            constraint.normal.dot(&constraint.weighted_normal1) +
            constraint.rot_axis1.dot(&constraint.weighted_rot_axis1);
    }

    if rb2.can_move() {
        // rotation axis
        constraint.weighted_normal2   = constraint.normal * rb2.inv_mass();
        constraint.rot_axis2          = (center - *rb2.center_of_mass()).cross(&constraint.normal);

        constraint.weighted_rot_axis2 = rb2.inv_inertia().transform(&constraint.rot_axis2);

        constraint.inv_projected_mass = constraint.inv_projected_mass +
            constraint.normal.dot(&constraint.weighted_normal2) +
            constraint.rot_axis2.dot(&constraint.weighted_rot_axis2);
    }

    constraint.inv_projected_mass = One::one::<N>() / constraint.inv_projected_mass;
}

fn fill_velocity_constraint<LV: VecExt<N> + Cross<AV> + Clone,
                            AV: Vec<N> + Clone,
                            N:  Num + Orderable + Bounded + Clone,
                            M:  Transform<LV> + Rotate<LV> + One,
                            II: Transform<AV> + Mul<II, II> + Clone + InertiaTensor<N, LV, M>>(
                            dt:              N,
                            normal:          LV,
                            center:          LV,
                            restitution:     N,
                            depth:           N,
                            initial_impulse: N,
                            lobound:         N,
                            hibound:         N,
                            rb1:             &RigidBody<N, LV, AV, M, II>,
                            rb2:             &RigidBody<N, LV, AV, M, II>,
                            constraint:      &mut VelocityConstraint<LV, AV, N>,
                            correction:      &CorrectionParameters<N>) {
    fill_constraint_geometry(normal, center.clone(), rb1, rb2, constraint);

    /*
     * Fill indice
     */
    constraint.id1 = rb1.index();
    constraint.id2 = rb2.index();

    /*
     * correction amount
     */
    constraint.objective = Zero::zero();

    if rb1.can_move() {
        constraint.objective = constraint.objective +
            - (rb1.lin_vel() + rb1.lin_acc() * dt).dot(&constraint.normal)
            + (rb1.ang_vel() + rb1.ang_acc() * dt).dot(&constraint.rot_axis1);
    }

    if rb2.can_move() {
        constraint.objective = constraint.objective +
            (rb2.lin_vel() + rb2.lin_acc() * dt).dot(&constraint.normal)
            + (rb2.ang_vel() + rb2.ang_acc() * dt).dot(&constraint.rot_axis2);
    }

    if constraint.objective < -correction.rest_eps {
        constraint.objective = constraint.objective + restitution * constraint.objective
    }

    constraint.objective = -constraint.objective;

    if depth < Zero::zero() {
        constraint.objective =  constraint.objective + depth / dt
    }
    else if depth < correction.corr_mode.max_depth_for_vel_corr() {
        constraint.objective = constraint.objective + depth * correction.corr_mode.vel_corr_factor() / dt
    }

    // for warm-starting
    constraint.impulse = initial_impulse;

    /*
     * constraint bounds
     */
    constraint.lobound = lobound;
    constraint.hibound = hibound;
}
