use std::cell::Ref;
use na::{self,  Row, Bounded};
use ncollide::math::Scalar;
use math::{Point, Vector};
use object::RigidBody;
use detection::joint::{Anchor, BallInSocket, Joint};
use resolution::constraint::velocity_constraint::VelocityConstraint;
use resolution::constraint::contact_equation::CorrectionParameters;
use resolution::constraint::contact_equation;

pub fn fill_second_order_equation<N: Scalar>(dt:          N,
                                             joint:       &BallInSocket<N>,
                                             constraints: &mut [VelocityConstraint<N>],
                                             correction:  &CorrectionParameters<N>) {
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
pub fn cancel_relative_linear_motion<N: Scalar, P>(
                                     dt:          N,
                                     global1:     &Point<N>,
                                     global2:     &Point<N>,
                                     anchor1:     &Anchor<N, P>,
                                     anchor2:     &Anchor<N, P>,
                                     constraints: &mut [VelocityConstraint<N>],
                                     correction:  &CorrectionParameters<N>) {
    let error      = (*global2 - *global1) * correction.joint_corr;
    let rot_axis1  = na::cross_matrix(&(*global1 - anchor1.center_of_mass()));
    let rot_axis2  = na::cross_matrix(&(*global2 - anchor2.center_of_mass()));

    for i in 0usize .. na::dimension::<Vector<N>>() {
        let mut lin_axis: Vector<N> = na::zero();
        let constraint = &mut constraints[i];

        lin_axis[i] = na::one();

        let opt_rb1 = write_anchor_id(anchor1, &mut constraint.id1);
        let opt_rb2 = write_anchor_id(anchor2, &mut constraint.id2);

        // XXX:
        // We use this dimension-dependent assignation.
        // This is needed because nalgebra does not define the `column` operator for vectors.
        // The true formula should be:
        // let rot_axis1 = -rot_axis1.col(i);
        // let rot_axis2 = rot_axis2.col(i);
        let (rot_axis1, rot_axis2) =
            if na::dimension::<Vector<N>>() == 2 {
                (-rot_axis1.row(i), rot_axis2.row(i))
            }
            else { // == 3
                (rot_axis1.row(i), -rot_axis2.row(i))
            };

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

        let _max: N = Bounded::max_value();
        constraint.lobound   = -_max;
        constraint.hibound   = _max;
        constraint.objective = -dvel - error[i] / dt;
        constraint.impulse   = na::zero(); // FIXME: cache
    }
}

#[inline]
pub fn write_anchor_id<'a, N: Scalar, P>(anchor: &'a Anchor<N, P>, id: &mut isize) -> Option<Ref<'a, RigidBody<N>>> {
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
