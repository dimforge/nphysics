use std::num::Bounded;
use nalgebra::na::{Translation, Rotation};
use nalgebra::na;
use ncollide::math::{Scalar, Vect, Orientation, Matrix};
use detection::joint::Fixed;
use detection::joint::Anchor;
use resolution::constraint::ball_in_socket_equation;
use resolution::constraint::velocity_constraint::VelocityConstraint;
use resolution::constraint::contact_equation::CorrectionParameters;
use resolution::constraint::contact_equation;

pub fn fill_second_order_equation(dt:          Scalar,
                                  joint:       &Fixed,
                                  constraints: &mut [VelocityConstraint],
                                  correction:  &CorrectionParameters) {
    let ref1 = joint.anchor1_pos();
    let ref2 = joint.anchor2_pos();

    ball_in_socket_equation::cancel_relative_linear_motion(
        dt.clone(),
        &ref1.translation(),
        &ref2.translation(),
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
        constraints.mut_slice_from(na::dim::<Vect>()),
        correction);
}

pub fn cancel_relative_angular_motion<P>(dt:          Scalar,
                                         ref1:        &Matrix,
                                         ref2:        &Matrix,
                                         anchor1:     &Anchor<P>,
                                         anchor2:     &Anchor<P>,
                                         constraints: &mut [VelocityConstraint],
                                         correction:  &CorrectionParameters) {
    let delta     = na::inv(ref2).expect("ref2 must be inversible.") * *ref1;
    let delta_rot = delta.rotation();

    let mut i = 0;
    na::canonical_basis(|rot_axis: Orientation| {
        let constraint = &mut constraints[i];

        let opt_rb1 = ball_in_socket_equation::write_anchor_id(anchor1, &mut constraint.id1);
        let opt_rb2 = ball_in_socket_equation::write_anchor_id(anchor2, &mut constraint.id2);

        contact_equation::fill_constraint_geometry(
            na::zero(),
            rot_axis.clone(),
            -rot_axis,
            &opt_rb1,
            &opt_rb2,
            constraint
        );

        let ang_vel1 = match opt_rb1 { Some(rb) => rb.get().ang_vel(), None => na::zero() };
        let ang_vel2 = match opt_rb2 { Some(rb) => rb.get().ang_vel(), None => na::zero() };

        let _M: Scalar = Bounded::max_value();
        constraint.lobound   = -_M;
        constraint.hibound   = _M;
        // FIXME: dont compute the difference at each iteration
        let error = na::dot(&delta_rot, &rot_axis) * correction.joint_corr / dt;
        constraint.objective = na::dot(&(ang_vel2 - ang_vel1), &rot_axis) - error;
        constraint.impulse   = na::zero(); // FIXME: cache

        i = i + 1;

        true
    })
}
