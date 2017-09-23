use num::Bounded;

use alga::general::Real;
use na::{self, U1};
use math::{Point, Vector};
use utils::GeneralizedCross;
use object::RigidBody;
use world::RigidBodyStorage;
use detection::joint::{Anchor, BallInSocket, Joint};
use resolution::constraint::velocity_constraint::VelocityConstraint;
use resolution::constraint::contact_equation::CorrectionParameters;
use resolution::constraint::contact_equation;

pub fn fill_second_order_equation<N: Real>(dt:          N,
                                           joint:       &BallInSocket<N>,
                                           constraints: &mut [VelocityConstraint<N>],
                                           correction:  &CorrectionParameters<N>,
                                           bodies:      &RigidBodyStorage<N>) {
    cancel_relative_linear_motion(
        dt,
        &joint.anchor1().pos(bodies),
        &joint.anchor2().pos(bodies),
        joint.anchor1(),
        joint.anchor2(),
        constraints,
        correction,
        bodies);
}

// FIXME: move this on another file. Something like "joint_equation_helper.rs"
pub fn cancel_relative_linear_motion<N: Real, P>(
                                     dt:          N,
                                     global1:     &Point<N>,
                                     global2:     &Point<N>,
                                     anchor1:     &Anchor<P>,
                                     anchor2:     &Anchor<P>,
                                     constraints: &mut [VelocityConstraint<N>],
                                     correction:  &CorrectionParameters<N>,
                                     bodies:      &RigidBodyStorage<N>) {
    let error      = (*global2 - *global1) * correction.joint_corr;
    let rot_axis1  = (*global1 - anchor1.center_of_mass(bodies)).gcross_matrix();
    let rot_axis2  = (*global2 - anchor2.center_of_mass(bodies)).gcross_matrix();

    for i in 0usize .. na::dimension::<Vector<N>>() {
        let mut lin_axis: Vector<N> = na::zero();
        let constraint = &mut constraints[i];

        lin_axis[i] = na::one();

        let opt_rb1 = write_anchor_id(anchor1, &mut constraint.id1, bodies);
        let opt_rb2 = write_anchor_id(anchor2, &mut constraint.id2, bodies);

        let rot_axis1 = -rot_axis1.fixed_columns::<U1>(i);
        let rot_axis2 =  rot_axis2.fixed_columns::<U1>(i).into_owned();

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

        let _max: N = Bounded::max_value();
        constraint.lobound   = -_max;
        constraint.hibound   = _max;
        constraint.objective = -dvel - error[i] / dt;
        constraint.impulse   = na::zero(); // FIXME: cache
    }
}

#[inline]
pub fn write_anchor_id<'a, N: Real, P>(anchor: &'a Anchor<P>, id: &mut isize, bodies: &'a RigidBodyStorage<N>) -> Option<&'a RigidBody<N>> {
    match anchor.body {
        Some(b) => {
            let rb = &bodies[b];
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
