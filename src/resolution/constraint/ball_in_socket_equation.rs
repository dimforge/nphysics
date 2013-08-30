use std::num::{Zero, One};
use nalgebra::traits::dim::Dim;
use nalgebra::traits::vector::{Vec, VecExt};
use nalgebra::traits::cross::{Cross, CrossMatrix};
use nalgebra::traits::row::Row;
use nalgebra::traits::transformation::Transform;
use nalgebra::traits::rotation::Rotate;
use object::body::ToRigidBody;
use object::volumetric::InertiaTensor;
use detection::joint::ball_in_socket::BallInSocket;
use resolution::constraint::velocity_constraint::VelocityConstraint;
use resolution::constraint::contact_equation::CorrectionMode;
use resolution::constraint::contact_equation;

pub fn fill_second_order_equation<N:  Num + Bounded + Clone,
                                  LV: VecExt<N> + CrossMatrix<CM> + Cross<AV> + Clone,
                                  AV: Vec<N> + Clone,
                                  M:  Transform<LV> + Rotate<LV> + One,
                                  II: Mul<II, II> + Transform<AV> + InertiaTensor<N, LV, M> + Clone,
                                  CM: Row<AV>>(
                                  dt:          N,
                                  joint:       &BallInSocket<N, LV, AV, M, II>,
                                  constraints: &mut [VelocityConstraint<LV, AV, N>],
                                  correction:  &CorrectionMode<N>) {
    let global1    = joint.anchor1_pos();
    let global2    = joint.anchor2_pos();
    let error      = (global2 - global1) * correction.vel_corr_factor();
    let rot_axis1  = (global1 - joint.center_of_mass1()).cross_matrix();
    let rot_axis2  = (global2 - joint.center_of_mass2()).cross_matrix();

    for i in range(0u, Dim::dim(None::<LV>)) {
        let mut lin_axis: LV = Zero::zero();
        let constraint = &mut constraints[i];

        lin_axis.set(i, One::one());

        let opt_rb1;
        let opt_rb2;

        match joint.anchor1().body {
            Some(b) => {
                if b.can_move() {
                    opt_rb1 = Some(&*b.to_rigid_body_or_fail());
                    constraint.id1 = b.index()
                }
                else {
                    opt_rb1 = None;
                    constraint.id1 = -1
                }
            },
            None => { opt_rb1 = None; constraint.id1 = -1 }
        }

        match joint.anchor2().body {
            Some(b) => {
                if b.can_move() {
                    opt_rb2 = Some(&*b.to_rigid_body_or_fail());
                    constraint.id2 = b.index()
                }
                else {
                    opt_rb2 = None;
                    constraint.id2 = -1
                }
            },
            None => { opt_rb2 = None; constraint.id2 = -1 }
        }

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
        constraint.impulse   = Zero::zero(); // FIXME: cache
    }
}
