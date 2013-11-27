use nalgebra::na::{CrossMatrix, Row};
use nalgebra::na;
use detection::joint::fixed::Fixed;
use detection::joint::anchor::Anchor;
use resolution::constraint::ball_in_socket_equation;
use resolution::constraint::velocity_constraint::VelocityConstraint;
use resolution::constraint::contact_equation::CorrectionParameters;
use resolution::constraint::contact_equation;
use aliases::traits::{NPhysicsScalar, NPhysicsDirection, NPhysicsOrientation, NPhysicsTransform,
                      NPhysicsInertia};

pub fn fill_second_order_equation<N:  Clone + NPhysicsScalar,
                                  LV: Clone + NPhysicsDirection<N, AV> + CrossMatrix<CM>,
                                  AV: Clone + NPhysicsOrientation<N>,
                                  M:  Clone + NPhysicsTransform<LV, AV>,
                                  II: Clone + NPhysicsInertia<N, LV, AV, M>,
                                  CM: Row<AV>>(
                                  dt:          N,
                                  joint:       &Fixed<N, LV, AV, M, II>,
                                  constraints: &mut [VelocityConstraint<LV, AV, N>],
                                  correction:  &CorrectionParameters<N>) {
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
        constraints.mut_slice_from(na::dim::<LV>()),
        correction);
}

pub fn cancel_relative_angular_motion<N:  Clone + NPhysicsScalar,
                                      LV: Clone + NPhysicsDirection<N, AV> + CrossMatrix<CM>,
                                      AV: Clone + NPhysicsOrientation<N>,
                                      M:  Clone + NPhysicsTransform<LV, AV>,
                                      II: Clone + NPhysicsInertia<N, LV, AV, M>,
                                      CM: Row<AV>,
                                      P>(
                                      dt:          N,
                                      ref1:        &M,
                                      ref2:        &M,
                                      anchor1:     &Anchor<N, LV, AV, M, II, P>,
                                      anchor2:     &Anchor<N, LV, AV, M, II, P>,
                                      constraints: &mut [VelocityConstraint<LV, AV, N>],
                                      correction:  &CorrectionParameters<N>) {
    let delta     = na::inv(ref2).expect("ref2 must be inversible.") * *ref1;
    let delta_rot = delta.rotation();

    let mut i = 0;
    na::canonical_basis(|rot_axis: AV| {
        let constraint = &mut constraints[i];

        let opt_b1 = ball_in_socket_equation::write_anchor_id(anchor1, &mut constraint.id1);
        let opt_b2 = ball_in_socket_equation::write_anchor_id(anchor2, &mut constraint.id2);
        let opt_rb1 = match opt_b1 { Some(b) => Some(b.to_rigid_body_or_fail()), None => None };
        let opt_rb2 = match opt_b2 { Some(b) => Some(b.to_rigid_body_or_fail()), None => None };

        contact_equation::fill_constraint_geometry(
            na::zero(),
            rot_axis.clone(),
            -rot_axis,
            opt_rb1,
            opt_rb2,
            constraint
        );

        let ang_vel1 = match opt_rb1 { Some(rb) => rb.ang_vel(), None => na::zero() };
        let ang_vel2 = match opt_rb2 { Some(rb) => rb.ang_vel(), None => na::zero() };

        let _M: N = Bounded::max_value();
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
