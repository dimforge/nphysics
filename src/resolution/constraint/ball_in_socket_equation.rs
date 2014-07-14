use std::num::Bounded;
use std::cell::Ref;
use nalgebra::na::{Row, Indexable};
use nalgebra::na::Mat3;
use nalgebra::na::Vec2;
use nalgebra::na;
use ncollide::math::{Scalar, Vect};
use ncollide::math::Orientation;
use object::RigidBody;
use detection::joint::{Anchor, BallInSocket, Joint};
use resolution::constraint::velocity_constraint::VelocityConstraint;
use resolution::constraint::contact_equation::CorrectionParameters;
use resolution::constraint::contact_equation;

pub fn fill_second_order_equation(dt:          Scalar,
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
                                     dt:          Scalar,
                                     global1:     &Vect,
                                     global2:     &Vect,
                                     anchor1:     &Anchor<P>,
                                     anchor2:     &Anchor<P>,
                                     constraints: &mut [VelocityConstraint],
                                     correction:  &CorrectionParameters) {
    let error      = (global2 - *global1) * correction.joint_corr;
    let rot_axis1  = na::cross_matrix(&(global1 - anchor1.center_of_mass()));
    let rot_axis2  = na::cross_matrix(&(global2 - anchor2.center_of_mass()));

    for i in range(0u, na::dim::<Vect>()) {
        let mut lin_axis: Vect = na::zero();
        let constraint = &mut constraints[i];

        lin_axis.set(i, na::one());

        let opt_rb1 = write_anchor_id(anchor1, &mut constraint.id1);
        let opt_rb2 = write_anchor_id(anchor2, &mut constraint.id2);

        // XXX:
        // We create this dimension-based dispatch.
        // This is needed because nalgebra does not define the `column` operator for vectors.
        // The true formula should be:
        // let rot_axis1 = -rot_axis1.col(i);
        // let rot_axis2 = rot_axis2.col(i);
        #[dim3]
        #[inline(always)]
        fn rot_axis(rot_axis1: &Mat3<Scalar>, rot_axis2: &Mat3<Scalar>, i: uint) -> (Orientation, Orientation){
            (rot_axis1.row(i), -rot_axis2.row(i))
        }

        #[dim2]
        #[inline(always)]
        fn rot_axis(rot_axis1: &Vec2<Scalar>, rot_axis2: &Vec2<Scalar>, i: uint) -> (Orientation, Orientation) {
            (-rot_axis1.row(i), rot_axis2.row(i))
        }

        let (rot_axis1, rot_axis2) = rot_axis(&rot_axis1, &rot_axis2, i);

        let dvel = contact_equation::relative_velocity(
            &opt_rb1.as_ref().map(|r| &**r),
            &opt_rb2.as_ref().map(|r| &**r),
            &lin_axis, 
            &rot_axis1,
            &rot_axis2,
            &dt);

        contact_equation::fill_constraint_geometry(
            lin_axis,
            rot_axis1,
            rot_axis2,
            &opt_rb1.as_ref().map(|r| &**r),
            &opt_rb2.as_ref().map(|r| &**r),
            constraint
        );

        let _M: Scalar = Bounded::max_value();
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
            let rb = b.borrow();
            let can_move;
            let rid;

            {
                can_move = rb.can_move();
                rid      = rb.index();
            }

            if can_move {
                *id = rid;

                Some(rb)
            }
            else {
                *id = -1;

                None
            }
        },
        None => { *id = -1; None }
    }
}
