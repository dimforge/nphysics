pub fn fill_second_order_contact_equation(&mut self,
                                          joint:       &BallInSocketJoint<V>,
                                          constraints: &mut ~[VelocityConstraint<LV, AV, N>]) {
    let error = global2 - global1;

    let rot_axis1 = (global1 - com1).cross_matrix();
    let rot_axis2 = (global2 - com2).cross_matrix();

    for i in range(0u, Dim::dim::<V>()) {
        let mut lin_axis = Zero::zero::<V>();

        lin_axis.set(i, One::one());

        match *joint.anchor1().body {
            Some(b) => constraint[i].id1 = b.index(),
            None    => constraint[i].id1 = -1
        }

        match *joint.anchor2().body {
            Some(b) => constraint[i].id2 = b.index(),
            None    => constraint[i].id2 = -1
        }

        constraint[i].normal    = lin_axis;
        constraint[i].rot_axis1 = rot_axis1.row(i);
        constraint[i].rot_axis2 = -rot_axis2.row(i);
        constraint[i].lobound   = -Bounded::max_value::<N>();
        constraint[i].hibound   = Bounded::max_value::<N>();
        constraint[i].objective = -error.at(i) / dt; // XXX - relative velocity
        constraint[i].impulse   = Zero::zero(); // FIXME: cache
    }
}
