use nalgebra::na::{CrossMatrix, Row};
use nalgebra::na;
use object::Body;
use detection::joint::anchor::Anchor;
use detection::joint::ball_in_socket::BallInSocket;
use resolution::constraint::velocity_constraint::VelocityConstraint;
use resolution::constraint::contact_equation::CorrectionMode;
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
                                  joint:       &BallInSocket<N, LV, AV, M, II>,
                                  constraints: &mut [VelocityConstraint<LV, AV, N>],
                                  correction:  &CorrectionMode<N>) {
    cancel_relative_linear_motion(
        dt,
        &joint.anchor1_pos(),
        &joint.anchor2_pos(),
        joint.anchor1(),
        joint.anchor2(),
        constraints,
        correction);
}

// FIXME: move this on another file. Something like "joint_equation_helper.rs"
pub fn cancel_relative_linear_motion<N:  Clone + NPhysicsScalar,
                                     LV: Clone + NPhysicsDirection<N, AV> + CrossMatrix<CM>,
                                     AV: Clone + NPhysicsOrientation<N>,
                                     M:  Clone + NPhysicsTransform<LV, AV>,
                                     II: Clone + NPhysicsInertia<N, LV, AV, M>,
                                     CM: Row<AV>,
                                     P>(
                                     dt:          N,
                                     global1:     &LV,
                                     global2:     &LV,
                                     anchor1:     &Anchor<N, LV, AV, M, II, P>,
                                     anchor2:     &Anchor<N, LV, AV, M, II, P>,
                                     constraints: &mut [VelocityConstraint<LV, AV, N>],
                                     correction:  &CorrectionMode<N>) {
    let error      = (global2 - *global1) * correction.vel_corr_factor();
    let rot_axis1  = na::cross_matrix(&(global1 - anchor1.center_of_mass()));
    let rot_axis2  = na::cross_matrix(&(global2 - anchor2.center_of_mass()));

    for i in range(0u, na::dim::<LV>()) {
        let mut lin_axis: LV = na::zero();
        let constraint = &mut constraints[i];

        lin_axis.set(i, na::one());

        let opt_b1 = write_anchor_id(anchor1, &mut constraint.id1);
        let opt_b2 = write_anchor_id(anchor2, &mut constraint.id2);
        let opt_rb1 = match opt_b1 { Some(b) => Some(b.to_rigid_body_or_fail()), None => None };
        let opt_rb2 = match opt_b2 { Some(b) => Some(b.to_rigid_body_or_fail()), None => None };

        let rot_axis1 = rot_axis1.row(i);
        let rot_axis2 = -rot_axis2.row(i);

        let dvel = contact_equation::relative_velocity(
            opt_rb1,
            opt_rb2,
            &lin_axis, 
            &rot_axis1,
            &rot_axis2,
            &dt);

        contact_equation::fill_constraint_geometry(
            lin_axis,
            rot_axis1,
            rot_axis2,
            opt_rb1,
            opt_rb2,
            constraint
        );

        let _M: N = Bounded::max_value();
        constraint.lobound   = -_M;
        constraint.hibound   = _M;
        constraint.objective = -dvel - error.at(i) / dt;
        constraint.impulse   = na::zero(); // FIXME: cache
    }
}

#[inline]
pub fn write_anchor_id<'r,
                       N:  Clone + NPhysicsScalar,
                       LV: Clone + NPhysicsDirection<N, AV>,
                       AV: Clone + NPhysicsOrientation<N>,
                       M:  NPhysicsTransform<LV, AV>,
                       II: Clone + NPhysicsInertia<N, LV, AV, M>,
                       P>(
                       anchor: &'r Anchor<N, LV, AV, M, II, P>,
                       id:     &mut int)
                       -> Option<@mut Body<N, LV, AV, M, II>> {
    match anchor.body {
        Some(b) => {
            if b.can_move() {
                *id = b.index();

                Some(b)
            }
            else {
                *id = -1;

                None
            }
        },
        None => { *id = -1; None }
    }
}
