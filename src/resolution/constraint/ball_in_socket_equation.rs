use std::cell::Ref;
use nalgebra::na::{Row, Indexable};
use nalgebra::na;
use ncollide::math::{N, LV};
use object::RigidBody;
use detection::joint::{Anchor, BallInSocket};
use resolution::constraint::velocity_constraint::VelocityConstraint;
use resolution::constraint::contact_equation::CorrectionParameters;
use resolution::constraint::contact_equation;

pub fn fill_second_order_equation(dt:          N,
                                  joint:       &BallInSocket,
                                  constraints: &mut [VelocityConstraint],
                                  correction:  &CorrectionParameters) {
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
pub fn cancel_relative_linear_motion<P>(
                                     dt:          N,
                                     global1:     &LV,
                                     global2:     &LV,
                                     anchor1:     &Anchor<P>,
                                     anchor2:     &Anchor<P>,
                                     constraints: &mut [VelocityConstraint],
                                     correction:  &CorrectionParameters) {
    let error      = (global2 - *global1) * correction.joint_corr;
    let rot_axis1  = na::cross_matrix(&(global1 - anchor1.center_of_mass()));
    let rot_axis2  = na::cross_matrix(&(global2 - anchor2.center_of_mass()));

    for i in range(0u, na::dim::<LV>()) {
        let mut lin_axis: LV = na::zero();
        let constraint = &mut constraints[i];

        lin_axis.set(i, na::one());

        let opt_rb1 = write_anchor_id(anchor1, &mut constraint.id1);
        let opt_rb2 = write_anchor_id(anchor2, &mut constraint.id2);

        let rot_axis1 = rot_axis1.row(i);
        let rot_axis2 = -rot_axis2.row(i);

        let dvel = contact_equation::relative_velocity(
            &opt_rb1,
            &opt_rb2,
            &lin_axis, 
            &rot_axis1,
            &rot_axis2,
            &dt);

        contact_equation::fill_constraint_geometry(
            lin_axis,
            rot_axis1,
            rot_axis2,
            &opt_rb1,
            &opt_rb2,
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
pub fn write_anchor_id<'a, P>(anchor: &'a Anchor<P>, id: &mut int) -> Option<Ref<'a, RigidBody>> {
    match anchor.body {
        Some(ref b) => {
            let brb      = b.borrow().borrow();
            let can_move;
            let rid;

            {
                let rb   = brb.get();
                can_move = rb.can_move();
                rid      = rb.index();
            }

            if can_move {
                *id = rid;

                Some(brb)
            }
            else {
                *id = -1;

                None
            }
        },
        None => { *id = -1; None }
    }
}
