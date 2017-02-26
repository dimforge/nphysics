use num::Bounded;
use alga::general::Real;
use alga::linear::FiniteDimVectorSpace;
use na;
use detection::joint::{Fixed, Anchor, Joint};
use resolution::constraint::ball_in_socket_equation;
use resolution::constraint::velocity_constraint::VelocityConstraint;
use resolution::constraint::contact_equation::CorrectionParameters;
use resolution::constraint::contact_equation;
use math::{Vector, Point, Orientation, Isometry};

pub fn fill_second_order_equation<N: Real>(dt:          N,
                                           joint:       &Fixed<N>,
                                           constraints: &mut [VelocityConstraint<N>],
                                           correction:  &CorrectionParameters<N>) {
    let ref1 = joint.anchor1_pos();
    let ref2 = joint.anchor2_pos();

    ball_in_socket_equation::cancel_relative_linear_motion(
        dt,
        &Point::from_coordinates(ref1.translation.vector),
        &Point::from_coordinates(ref2.translation.vector),
        joint.anchor1(),
        joint.anchor2(),
        constraints,
        correction);

    cancel_relative_angular_motion(
        dt,
        &ref1,
        &ref2,
        joint.anchor1(),
        joint.anchor2(),
        &mut constraints[na::dimension::<Vector<N>>() ..],
        correction);
}

pub fn cancel_relative_angular_motion<N: Real, P>(dt:          N,
                                                  ref1:        &Isometry<N>,
                                                  ref2:        &Isometry<N>,
                                                  anchor1:     &Anchor<N, P>,
                                                  anchor2:     &Anchor<N, P>,
                                                  constraints: &mut [VelocityConstraint<N>],
                                                  correction:  &CorrectionParameters<N>) {
    let delta      = ref2.rotation.inverse() * ref1.rotation;
    let delta_axis = delta.scaled_axis();

    let mut i = 0;
    Orientation::canonical_basis(|rot_axis: &Orientation<N>| {
        let constraint = &mut constraints[i];

        let opt_rb1 = ball_in_socket_equation::write_anchor_id(anchor1, &mut constraint.id1);
        let opt_rb2 = ball_in_socket_equation::write_anchor_id(anchor2, &mut constraint.id2);

        contact_equation::fill_constraint_geometry(
            na::zero(),
            rot_axis.clone(),
            -rot_axis,
            &opt_rb1.as_ref().map(|r| &**r),
            &opt_rb2.as_ref().map(|r| &**r),
            constraint
        );

        let ang_vel1 = match opt_rb1 { Some(rb) => rb.ang_vel(), None => na::zero() };
        let ang_vel2 = match opt_rb2 { Some(rb) => rb.ang_vel(), None => na::zero() };

        let _max: N = Bounded::max_value();
        constraint.lobound = -_max;
        constraint.hibound = _max;
        // FIXME: dont compute the difference at each iteration
        let error = na::dot(&delta_axis, &rot_axis) * correction.joint_corr / dt;
        constraint.objective = na::dot(&(ang_vel2 - ang_vel1), &rot_axis) - error;
        constraint.impulse   = na::zero(); // FIXME: cache

        i = i + 1;

        true
    })
}
