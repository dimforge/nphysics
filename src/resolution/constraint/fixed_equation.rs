use std::num::{Zero, One};
use nalgebra::na::{
    AlgebraicVecExt, VecExt, Cross, CrossMatrix, Dim, Basis,
    Rotate, Transform, Translation, Rotation, Row, Inv
};
use object::volumetric::InertiaTensor;
use detection::joint::fixed::Fixed;
use detection::joint::anchor::Anchor;
use resolution::constraint::ball_in_socket_equation;
use resolution::constraint::velocity_constraint::VelocityConstraint;
use resolution::constraint::contact_equation::CorrectionMode;
use resolution::constraint::contact_equation;

pub fn fill_second_order_equation<N:  Num + Bounded + Clone,
                                  LV: VecExt<N> + CrossMatrix<CM> + Cross<AV> + Clone,
                                  AV: AlgebraicVecExt<N> + Clone,
                                  M:  Transform<LV> + Rotate<LV> + Mul<M, M> + Clone +
                                      Translation<LV> + Rotation<AV> + Inv + One,
                                  II: Mul<II, II> + InertiaTensor<N, LV, AV, M> + Clone,
                                  CM: Row<AV>>(
                                  dt:          N,
                                  joint:       &Fixed<N, LV, AV, M, II>,
                                  constraints: &mut [VelocityConstraint<LV, AV, N>],
                                  correction:  &CorrectionMode<N>) {
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
        constraints.mut_slice_from(Dim::dim(None::<LV>)),
        correction);
}

pub fn cancel_relative_angular_motion<N:  Num + Bounded + Clone,
                                     LV: VecExt<N> + CrossMatrix<CM> + Cross<AV> + Clone,
                                     AV: AlgebraicVecExt<N> + Clone,
                                     M:  Transform<LV> + Rotate<LV> + Rotation<AV> + Inv + Mul<M, M> + One,
                                     II: Mul<II, II> + InertiaTensor<N, LV, AV, M> + Clone,
                                     CM: Row<AV>,
                                     P>(
                                     dt:          N,
                                     ref1:        &M,
                                     ref2:        &M,
                                     anchor1:     &Anchor<N, LV, AV, M, II, P>,
                                     anchor2:     &Anchor<N, LV, AV, M, II, P>,
                                     constraints: &mut [VelocityConstraint<LV, AV, N>],
                                     correction:  &CorrectionMode<N>) {
    let delta     = ref2.inverted().expect("ref2 must be inversible.") * *ref1;
    let delta_rot = delta.rotation();

    let mut i = 0;
    do Basis::canonical_basis |rot_axis: AV| {
        let constraint = &mut constraints[i];

        let opt_b1 = ball_in_socket_equation::write_anchor_id(anchor1, &mut constraint.id1);
        let opt_b2 = ball_in_socket_equation::write_anchor_id(anchor2, &mut constraint.id2);
        let opt_rb1 = match opt_b1 { Some(b) => Some(b.to_rigid_body_or_fail()), None => None };
        let opt_rb2 = match opt_b2 { Some(b) => Some(b.to_rigid_body_or_fail()), None => None };

        contact_equation::fill_constraint_geometry(
            Zero::zero(),
            rot_axis.clone(),
            -rot_axis,
            opt_rb1,
            opt_rb2,
            constraint
        );

        let ang_vel1 = match opt_rb1 { Some(rb) => rb.ang_vel(), None => Zero::zero() };
        let ang_vel2 = match opt_rb2 { Some(rb) => rb.ang_vel(), None => Zero::zero() };

        let _M: N = Bounded::max_value();
        constraint.lobound   = -_M;
        constraint.hibound   = _M;
        // FIXME: dont compute the difference at each iteration
        let error = delta_rot.dot(&rot_axis) * correction.vel_corr_factor() / dt;
        constraint.objective = (ang_vel2 - ang_vel1).dot(&rot_axis) - error;
        constraint.impulse   = Zero::zero(); // FIXME: cache

        i = i + 1;

        true
    }
}
