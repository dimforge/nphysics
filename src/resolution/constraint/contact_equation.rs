use std::num::{One, Zero, Orderable, Bounded};
use nalgebra::traits::dim::Dim;
use nalgebra::traits::basis::Basis;
use nalgebra::traits::cross::Cross;
use nalgebra::traits::division_ring::DivisionRing;
use nalgebra::traits::dot::Dot;
use nalgebra::traits::transformation::Transform;
use nalgebra::traits::translation::Translation;
use nalgebra::traits::rotation::Rotate;
use nalgebra::traits::vector_space::VectorSpace;
use ncollide::contact::Contact;
use resolution::constraint::velocity_constraint::VelocityConstraint;
use object::rigid_body::RigidBody;
use object::volumetric::InertiaTensor;

pub struct CorrectionParameters<N> {
    depth_limit: N,
    corr_factor: N,
    depth_eps:   N,
    rest_eps:    N
}

pub fn fill_first_order_contact_equation<LV: VectorSpace<N> + Cross<AV> + Dot<N> + Dim + Clone,
                                         AV: VectorSpace<N> + Dot<N>,
                                         N:  DivisionRing + Orderable + Bounded + NumCast + Clone,
                                         M: Translation<LV> + Transform<LV> + Rotate<LV> + One,
                                         II: Transform<AV> + Mul<II, II> + InertiaTensor<M> + Clone>(
                                         dt:          N,
                                         coll:        &Contact<N, LV>,
                                         rb1:         &RigidBody<N, LV, AV, M, II>,
                                         rb2:         &RigidBody<N, LV, AV, M, II>,
                                         constraint:  &mut VelocityConstraint<LV, AV, N>,
                                         correction:  &CorrectionParameters<N>) {
    let center = (coll.world1 + coll.world2).scalar_div(&NumCast::from(2.0f64));

    fill_constraint_geometry(coll.normal.clone(), center, rb1, rb2, constraint);

    /*
     * Fill indice
     */
    constraint.id1 = rb1.index();
    constraint.id2 = rb2.index();

    /*
     * Fill b
     */
    constraint.objective = correction.corr_factor *
        (coll.depth - correction.depth_limit).max(&Zero::zero()) / dt;

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
pub fn fill_second_order_contact_equation<LV: VectorSpace<N> + Cross<AV>  + Dot<N> + Basis + Dim +
                                              Clone,
                                          AV: VectorSpace<N> + Dot<N> + Clone,
                                          N:  DivisionRing + Orderable + Bounded + Signed + Clone +
                                              NumCast + ToStr,
                                          M:  Translation<LV> + Transform<LV> + Rotate<LV> + One,
                                          II: Transform<AV> + Mul<II, II> + InertiaTensor<M> +
                                              Clone>(
                                          dt:           N,
                                          coll:         &Contact<N, LV>,
                                          rb1:          &RigidBody<N, LV, AV, M, II>,
                                          rb2:          &RigidBody<N, LV, AV, M, II>,
                                          rconstraint:  &mut VelocityConstraint<LV, AV, N>,
                                          idr:          uint,
                                          fconstraints: &mut [VelocityConstraint<LV, AV, N>],
                                          idf:          uint,
                                          correction:   &CorrectionParameters<N>) {
    let err = correction.corr_factor *
        (coll.depth.min(&correction.depth_limit) - correction.depth_eps).max(&Zero::zero()) / dt;
    let restitution = rb1.restitution() * rb2.restitution();

    let center = (coll.world1 + coll.world2).scalar_div(&NumCast::from(2.0f64));

    fill_velocity_constraint(dt.clone(),
                             coll.normal.clone(),
                             center.clone(),
                             restitution,
                             err,
                             Zero::zero(), // coll.impulses[0].clone(),
                             Zero::zero(),
                             Bounded::max_value(),
                             correction.rest_eps.clone(),
                             rb1,
                             rb2,
                             rconstraint);


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
                                 Zero::zero(), // coll.impulses[i].clone(),
                                 Zero::zero(), // dont setup the limit now
                                 Zero::zero(), // dont setup the limit now
                                 correction.rest_eps.clone(),
                                 rb1,
                                 rb2,
                                 constraint);

        constraint.friction_coeff    = friction.clone();
        constraint.friction_limit_id = idr;
        i = i + 1;
    }
}

fn fill_constraint_geometry<LV: VectorSpace<N> + Cross<AV> + Dot<N> + Dim + Clone,
                            AV: VectorSpace<N> + Dot<N>,
                            N:  DivisionRing + Clone,
                            M:  Translation<LV> + Transform<LV> + Rotate<LV> + One,
                            II: Transform<AV> + Mul<II, II> + InertiaTensor<M> + Clone>(
                            normal:     LV,
                            center:     LV,
                            rb1:        &RigidBody<N, LV, AV, M, II>,
                            rb2:        &RigidBody<N, LV, AV, M, II>,
                            constraint: &mut VelocityConstraint<LV, AV, N>) {
    constraint.normal             = normal;
    constraint.inv_projected_mass = Zero::zero();

    if rb1.can_move() {
        // rotation axis
        constraint.weighted_normal1   = constraint.normal.scalar_mul(&rb1.inv_mass());
        constraint.rot_axis1          = (center - rb1.transform_ref().translation()).cross(&-constraint.normal);

        constraint.weighted_rot_axis1 = rb1.inv_inertia().transform_vec(&constraint.rot_axis1);

        constraint.inv_projected_mass = constraint.inv_projected_mass +
            constraint.normal.dot(&constraint.weighted_normal1) +
            constraint.rot_axis1.dot(&constraint.weighted_rot_axis1);
    }

    if rb2.can_move() {
        // rotation axis
        constraint.weighted_normal2   = constraint.normal.scalar_mul(&rb2.inv_mass());
        constraint.rot_axis2          = (center - rb2.transform_ref().translation()).cross(&constraint.normal);

        constraint.weighted_rot_axis2 = rb2.inv_inertia().transform_vec(&constraint.rot_axis2);

        constraint.inv_projected_mass = constraint.inv_projected_mass +
            constraint.normal.dot(&constraint.weighted_normal2) +
            constraint.rot_axis2.dot(&constraint.weighted_rot_axis2);
    }

    constraint.inv_projected_mass = One::one::<N>() / constraint.inv_projected_mass;
}

fn fill_velocity_constraint<LV: VectorSpace<N> + Cross<AV>  + Dot<N> + Basis + Dim + Clone,
                            AV: VectorSpace<N> + Dot<N> + Clone,
                            N:  DivisionRing + Orderable + Bounded + Clone,
                            M:  Translation<LV> + Transform<LV> + Rotate<LV> + One,
                            II: Transform<AV> + Mul<II, II> + Clone + InertiaTensor<M>>(
                            dt:              N,
                            normal:          LV,
                            center:          LV,
                            restitution:     N,
                            error:           N,
                            initial_impulse: N,
                            lobound:         N,
                            hibound:         N,
                            rest_eps:        N,
                            rb1:             &RigidBody<N, LV, AV, M, II>,
                            rb2:             &RigidBody<N, LV, AV, M, II>,
                            constraint:      &mut VelocityConstraint<LV, AV, N>) {
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
            - (rb1.lin_vel() + rb1.lin_acc().scalar_mul(&dt)).dot(&constraint.normal)
            + (rb1.ang_vel() + rb1.ang_acc().scalar_mul(&dt)).dot(&constraint.rot_axis1);
    }

    if rb2.can_move() {
        constraint.objective = constraint.objective +
            (rb2.lin_vel() + rb2.lin_acc().scalar_mul(&dt)).dot(&constraint.normal)
            + (rb2.ang_vel() + rb2.ang_acc().scalar_mul(&dt)).dot(&constraint.rot_axis2);
    }

    if constraint.objective < -rest_eps {
        constraint.objective = constraint.objective + restitution * constraint.objective
    }

    constraint.objective = error - constraint.objective;

    // for warm-starting
    constraint.impulse = initial_impulse;

    /*
     * constraint bounds
     */
    constraint.lobound = lobound;
    constraint.hibound = hibound;
}
