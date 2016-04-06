#[cfg(test)]
mod test {
    #[cfg(feature = "dim2")]
    use na;
    #[cfg(feature = "dim2")]
    use na::{Vec1, Vec2, Iso2};
    #[cfg(feature = "dim2")]
    use ncollide::shape::Cuboid;
    #[cfg(feature = "dim2")]
    use object::{ActivationState, RigidBody};
    #[cfg(feature = "dim2")]
    use world::World;

    /// Gravity tests
    #[cfg(feature = "dim2")]
    #[test]
    fn gravity2() {
        // world
        let mut world = World::new();

        // rigidbody with side length 2, area 4 and mass 4
        let geom   = Cuboid::new(Vec2::new(1.0, 1.0));
        let rb = RigidBody::new_dynamic(geom, 1.0, 0.3, 0.6, None);
        let rb_handle = world.add_body(rb.clone());

        // ensure it's at the origin
        let expected = &Iso2::new(na::zero(), na::zero());
        assert!(na::approx_eq(rb_handle.borrow().position(), expected),
                format!("Initial position should be at zero. Actual: {:?}, Expected: {:?}",
                        rb_handle.borrow().position(), expected));

        // set gravity
        let gravity = Vec2::new(0.0, 20.0);
        world.set_gravity(gravity);

        // remove body and trigger a rust panic
        //	world.remove_body(&rb_handle2);


        // add another body in same position triggers a panic in ncollide
        // 	let mut rb3 = rb.clone();
        // 	rb3.append_translation(&Vec2::new(0.0, 0.0));
        // 	let rb_handle3 = world.add_body(rb3);

        // simulate a huge time step
        // The result is physically not correct as the acceleration integration
        // happens at the beginning of the step and then the body moves with the
        // terminal velocity for the entire time duration. Therefore it moves farther
        // than it actually should. Use smaller time intervals to approximate more
        // realistic values (see below).
        let expected = &Iso2::new(Vec2::new(0.0, 20.0), na::zero());
        world.step(1.0);
        assert!(na::approx_eq(rb_handle.borrow().position(), expected),
                format!("Gravity did not pull object correctly (large time step). Actual: {:?}, Expected: {:?}",
                rb_handle.borrow().position(), expected));


        // reset the body
        rb_handle.borrow_mut().set_lin_vel(Vec2::new(0.0, 0.0));
        rb_handle.borrow_mut().set_transformation(Iso2::new(na::zero(), na::zero()));

        // simulate small time steps
        let expected = &Iso2::new(Vec2::new(0.0, 10.01), na::zero());
        for _ in 0 .. 1000 {
            world.step(0.001);
        }
        assert!(na::approx_eq_eps(rb_handle.borrow().position(), expected, &0.01),
                format!("Gravity did not pull object correctly (small time steps). Actual: {:?}, Expected: {:?}",
                rb_handle.borrow().position(), expected));


        // reset the body
        rb_handle.borrow_mut().set_lin_vel(Vec2::new(0.0, 20.0));
        rb_handle.borrow_mut().set_transformation(Iso2::new(na::zero(), na::zero()));

        // Switch off gravity globally
        let expected = &Iso2::new(Vec2::new(0.0, 20.0), na::zero());
        world.set_gravity(na::zero());
        for _ in 0 .. 1000 {
            world.step(0.001);
        }
        assert!(na::approx_eq_eps(rb_handle.borrow().position(), expected, &0.001),
                format!("Gravity did not correctly switch off (global). Actual: {:?}, Expected: {:?}",
                rb_handle.borrow().position(), expected));


        // reset the body
        rb_handle.borrow_mut().set_lin_vel(Vec2::new(0.0, 0.0));
        rb_handle.borrow_mut().set_transformation(Iso2::new(na::zero(), na::zero()));

        // Wait until body deactivates
        for _ in 0 .. 1000 {
            world.step(0.001);
        }

        assert!(*rb_handle.borrow().activation_state() == ActivationState::Inactive,
                format!("Body should be inactive by now, but is {:?}", rb_handle.borrow().activation_state()));
        rb_handle.borrow_mut().activate(1.0);
        assert!(*rb_handle.borrow().activation_state() != ActivationState::Inactive,
                format!("Body should be active by now, but is {:?}", rb_handle.borrow().activation_state()));

        // Changing gravity has to work
        let expected = &Iso2::new(Vec2::new(0.0, 10.0), na::zero());
        world.set_gravity(Vec2::new(0.0, 20.0));
        for _ in 0 .. 1000 {
            world.step(0.001);
        }

        world.set_gravity(Vec2::new(0.0, -20.0));
        for _ in 0 .. 2000 {
            world.step(0.001);
        }

        assert!(na::approx_eq_eps(rb_handle.borrow().position(), expected, &0.02),
                format!("Gravity did not change correctly. Actual: {:?}, Expected: {:?}",
                        rb_handle.borrow().position(), expected));
    }


    /// Forces tests
    #[cfg(feature = "dim2")]
    #[test]
    fn forces2() {
        // world
        let mut world = World::new();

        // rigidbody with side length 2, area 4 and mass 4
        let geom   = Cuboid::new(Vec2::new(1.0, 1.0));
        let rb = RigidBody::new_dynamic(geom.clone(), 1.0, 0.3, 0.6, None);
        let rb_handle = world.add_body(rb.clone());

        // add another body with double the density
        let mut rb2 = RigidBody::new_dynamic(geom.clone(), 2.0, 0.3, 0.6, None);
        rb2.append_translation(&Vec2::new(5.0, 0.0));
        let rb_handle2 = world.add_body(rb2);

        // switch off gravity
        world.set_gravity(Vec2::new(0.0, 0.0));

        // Force has to work for different masses
        // apply force
        rb_handle.borrow_mut().append_lin_force(Vec2::new(0.0, 10.0));
        rb_handle2.borrow_mut().append_lin_force(Vec2::new(0.0, 10.0));
        rb_handle.borrow_mut().set_deactivation_threshold( None );
        rb_handle2.borrow_mut().set_deactivation_threshold( None );

        // simulate
        for _ in 0 .. 2000 {
            world.step(0.001);
        }
        // mass of 4 (kg), force of 10 (N) => acc = force/mass = 2.5 (m/s^2)
        // distance after 2 secs: x = 1/2 * acc * t^2 = 5 (m)
        let expected = &Iso2::new(Vec2::new(0.0, 5.0), na::zero());
        assert!(na::approx_eq_eps(rb_handle.borrow().position(), expected, &0.01),
                format!("Force did not properly pull first body. Actual: {:?}, Expected: {:?}",
                        rb_handle.borrow().position(), expected));
        // mass of 8 (kg), force of 10 (N)=> acc = force/mass = 1.25 (m/s^2)
        // distance after 2 secs: x = 1/2 * acc * t^2 = 2.5 (m)
        let expected = &Iso2::new(Vec2::new(5.0, 2.5), na::zero());
        assert!(na::approx_eq_eps(rb_handle2.borrow().position(), expected, &0.01),
                format!("Force did not properly pull second body. Actual: {:?}, Expected: {:?}",
                        rb_handle2.borrow().position(), expected));


        // reset bodies
        rb_handle.borrow_mut().set_transformation(Iso2::new(na::zero(), na::zero()));
        rb_handle.borrow_mut().set_lin_vel(Vec2::new(0.0, 0.0));
        rb_handle2.borrow_mut().deactivate();

        // clearing forces has to work
        rb_handle.borrow_mut().clear_linear_force();
        // simulate
        for _ in 0 .. 1000 {
            world.step(0.001);
        }
        let expected = &Iso2::new(Vec2::new(0.0, 0.0), na::zero());
        assert!(na::approx_eq_eps(rb_handle.borrow().position(), expected, &0.01),
                format!("Force should have been cleared. Actual: {:?}, Expected: {:?}",
                        rb_handle.borrow().position(), expected));


        // adding forces has to work
        rb_handle.borrow_mut().append_lin_force(Vec2::new(0.0, 5.0));
        rb_handle.borrow_mut().append_lin_force(Vec2::new(-10.0, 5.0));
        // simulate
        for _ in 0 .. 2000 {
            world.step(0.001);
        }
        let expected = &Iso2::new(Vec2::new(-5.0, 5.0), na::zero());
        assert!(na::approx_eq_eps(rb_handle.borrow().position(), expected, &0.01),
                format!("Forces did not properly add up. Actual: {:?}, Expected: {:?}",
                        rb_handle.borrow().position(), expected));


        // angular force has to work correctly for different inertias and types
        // reset bodies
        rb_handle.borrow_mut().set_transformation(Iso2::new(na::zero(), na::zero()));
        rb_handle.borrow_mut().set_lin_vel(Vec2::new(0.0, 0.0));
        rb_handle.borrow_mut().clear_forces();
        rb_handle2.borrow_mut().set_transformation(Iso2::new(Vec2::new(5.0, 0.0), na::zero()));
        rb_handle2.borrow_mut().set_lin_vel(Vec2::new(0.0, 0.0));
        rb_handle2.borrow_mut().activate(1.0);
        rb_handle2.borrow_mut().clear_forces();

        // add angular forces
        rb_handle.borrow_mut().append_ang_force(Vec1::new(10.0));
        rb_handle2.borrow_mut().append_ang_force(Vec1::new(10.0));

        // simulate
        for _ in 0 .. 1000 {
            world.step(0.001);
        }
        // expected angle
        // force of force of 10 (N), inertia of 2.67 => acc = force/inertia = 3.75 rad/s^2
        // 1 sec acceleration => angle = 1/2 * acc * t^2 = 1.875
        let expected = &Iso2::new(na::zero(), Vec1::new(1.875));
        assert!(na::approx_eq_eps(rb_handle.borrow().position(), expected, &0.01),
                format!("Rotation did not properly work. Actual: {:?}, Expected: {:?}",
                        rb_handle.borrow().position(), expected));
        // expected angle
        // force of force of 10 (N), inertia of 5.33 => acc = force/inertia = 1.875 rad/s^2
        // 1 sec acceleration => angle = 1/2 * acc * t^2 = 0.9375
        let expected = &Iso2::new(Vec2::new(5.0, 0.0), Vec1::new(0.9375));
        assert!(na::approx_eq_eps(rb_handle2.borrow().position(), expected, &0.01),
                format!("Rotation2 did not properly work. Actual: {:?}, Expected: {:?}",
                        rb_handle2.borrow().position(), expected));

        // clear angular forces
        rb_handle.borrow_mut().set_transformation(Iso2::new(na::zero(), na::zero()));
        rb_handle.borrow_mut().set_ang_vel(Vec1::new(0.0));
        rb_handle2.borrow_mut().set_transformation(Iso2::new(Vec2::new(5.0, 0.0), na::zero()));
        rb_handle2.borrow_mut().set_ang_vel(Vec1::new(0.0));
        rb_handle.borrow_mut().clear_angular_force();
        rb_handle2.borrow_mut().clear_angular_force();

        // simulate
        for _ in 0 .. 1000 {
            world.step(0.001);
        }
        // expected angle
        let expected = &Iso2::new(na::zero(), Vec1::new(0.0));
        assert!(na::approx_eq_eps(rb_handle.borrow().position(), expected, &0.01),
                format!("Rotation did not properly stop. Actual: {:?}, Expected: {:?}",
                        rb_handle.borrow().position(), expected));
        // expected angle
        let expected = &Iso2::new(Vec2::new(5.0, 0.0), Vec1::new(0.0));
        assert!(na::approx_eq_eps(rb_handle2.borrow().position(), expected, &0.01),
                format!("Rotation2 did not properly stop. Actual: {:?}, Expected: {:?}",
                        rb_handle2.borrow().position(), expected));


        // reset bodies
        rb_handle.borrow_mut().set_transformation(Iso2::new(na::zero(), na::zero()));
        rb_handle.borrow_mut().set_ang_vel(Vec1::new(0.0));
        rb_handle.borrow_mut().clear_angular_force();
        rb_handle2.borrow_mut().deactivate();

        // add angular forces
        rb_handle.borrow_mut().append_ang_force(Vec1::new(10.0));
        rb_handle.borrow_mut().append_ang_force(Vec1::new(-20.0));

        // simulate
        for _ in 0 .. 1000 {
            world.step(0.001);
        }
        // expected angle
        // resulting force of force of -10 (N), inertia of 2.67 => acc = force/inertia = -3.75 rad/s^2
        // 1 sec acceleration => angle = 1/2 * acc * t^2 = -1.875
        let expected = &Iso2::new(na::zero(), Vec1::new(-1.875));
        assert!(na::approx_eq_eps(rb_handle.borrow().position(), expected, &0.01),
                format!("Combined forces rotation did not properly work. Actual: {:?}, Expected: {:?}",
                        rb_handle.borrow().position(), expected));


        // reset bodies
        rb_handle.borrow_mut().set_transformation(Iso2::new(na::zero(), na::zero()));
        rb_handle.borrow_mut().set_ang_vel(Vec1::new(0.0));
        rb_handle.borrow_mut().clear_angular_force();

        // set and clear both linear and angular forces
        rb_handle.borrow_mut().append_lin_force(Vec2::new(0.0, 10.0));
        rb_handle.borrow_mut().append_ang_force(Vec1::new(10.0));
        rb_handle.borrow_mut().clear_forces();

        // simulate
        for _ in 0 .. 1000 {
            world.step(0.001);
        }
        // expected result, body remains in the origin as all forces got cleared
        let expected = &Iso2::new(na::zero(), na::zero());
        assert!(na::approx_eq_eps(rb_handle.borrow().position(), expected, &0.01),
                format!("Cleared forces shouldn't work anymore. Actual: {:?}, Expected: {:?}",
                        rb_handle.borrow().position(), expected));

        // reset bodies
        rb_handle.borrow_mut().set_transformation(Iso2::new(na::zero(), na::zero()));
        rb_handle.borrow_mut().set_ang_vel(Vec1::new(0.0));
        rb_handle.borrow_mut().clear_angular_force();

        // only clear angular force
        rb_handle.borrow_mut().append_lin_force(Vec2::new(0.0, 10.0));
        rb_handle.borrow_mut().append_ang_force(Vec1::new(10.0));
        rb_handle.borrow_mut().clear_angular_force();

        // simulate
        for _ in 0 .. 2000 {
            world.step(0.001);
        }
        // expected result, body moves but doesn't rotate
        let expected = &Iso2::new(Vec2::new(0.0, 5.0), na::zero());
        assert!(na::approx_eq_eps(rb_handle.borrow().position(), expected, &0.01),
                format!("Only linear movement is expected. Actual: {:?}, Expected: {:?}",
                        rb_handle.borrow().position(), expected));


        // reset bodies
        rb_handle.borrow_mut().set_transformation(Iso2::new(na::zero(), na::zero()));
        rb_handle.borrow_mut().set_lin_vel(na::zero());
        rb_handle.borrow_mut().set_ang_vel(na::zero());
        rb_handle.borrow_mut().clear_forces();

        // only clear linear force
        rb_handle.borrow_mut().append_lin_force(Vec2::new(0.0, 10.0));
        rb_handle.borrow_mut().append_ang_force(Vec1::new(10.0));
        rb_handle.borrow_mut().clear_linear_force();

        // simulate
        for _ in 0 .. 1000 {
            world.step(0.001);
        }
        // expected result, body rotates but doesn't move
        let expected = &Iso2::new(na::zero(), Vec1::new(1.875));
        assert!(na::approx_eq_eps(rb_handle.borrow().position(), expected, &0.01),
                format!("Only rotation is expected. Actual: {:?}, Expected: {:?}",
                        rb_handle.borrow().position(), expected));


        // reset bodies
        rb_handle.borrow_mut().set_transformation(Iso2::new(na::zero(), na::zero()));
        rb_handle.borrow_mut().set_lin_vel(na::zero());
        rb_handle.borrow_mut().set_ang_vel(na::zero());
        rb_handle.borrow_mut().clear_forces();
        rb_handle.borrow_mut().activate(1.0);
        rb_handle2.borrow_mut().set_transformation(Iso2::new(Vec2::new(5.0, 0.0), na::zero()));
        rb_handle2.borrow_mut().set_lin_vel(na::zero());
        rb_handle2.borrow_mut().set_ang_vel(na::zero());
        rb_handle2.borrow_mut().clear_forces();
        rb_handle2.borrow_mut().activate(1.0);

        // apply force on point
        rb_handle.borrow_mut().append_force_wrt_point(Vec2::new(0.0, 10.0), Vec2::new(1.0, 1.0));
        rb_handle2.borrow_mut().append_force_wrt_point(Vec2::new(0.0, 10.0), Vec2::new(1.0, 1.0));

        // simulate
        for _ in 0 .. 1000 {
            world.step(0.001);
        }
        // expected result
        // linear displacement 1.25
        // angular rotation: -1.875
        let expected = &Iso2::new(Vec2::new(0.0, 1.25), Vec1::new(1.875));
        assert!(na::approx_eq_eps(rb_handle.borrow().position(), expected, &0.01),
                format!("Only rotation is expected on body 1. Actual: {:?}, Expected: {:?}",
                        rb_handle.borrow().position(), expected));
        // expected result
        // linear displacement 1.25
        // angular rotation: -1.875
        let expected = &Iso2::new(Vec2::new(5.0, 0.625), Vec1::new(0.9375));
        assert!(na::approx_eq_eps(rb_handle2.borrow().position(), expected, &0.01),
                format!("Only rotation is expected on body 2. Actual: {:?}, Expected: {:?}",
                        rb_handle2.borrow().position(), expected));
    }

    /// Impulse tests
    #[cfg(feature = "dim2")]
    #[test]
    fn impulse2() {
        // world
        let mut world = World::new();

        // rigidbody with side length 2, area 4 and mass 4
        let geom = Cuboid::new(Vec2::new(1.0, 1.0));
        let rb = RigidBody::new_dynamic(geom.clone(), 1.0, 0.3, 0.6, None);
        let rb_handle = world.add_body(rb.clone());

        // add another body with double the density
        let mut rb2 = RigidBody::new_dynamic(geom.clone(), 2.0, 0.3, 0.6, None);
        rb2.append_translation(&Vec2::new(5.0, 0.0));
        let rb_handle2 = world.add_body(rb2);

        // switch off gravity
        world.set_gravity(Vec2::new(0.0, 0.0));

        // impulses have to work for different masses
        rb_handle.borrow_mut().apply_central_impulse(Vec2::new(0.0, 10.0));
        rb_handle2.borrow_mut().apply_central_impulse(Vec2::new(0.0, 10.0));

        // simulate
        for _ in 0 .. 1000 {
            world.step(0.001);
        }
        // expected result
        // impulse of 10 N*s on body with mass 4 (kg) results in
        // velocity = impulse/mass = 10 N*s / 4 kg = 2.5 m/s
        // distance = velocity * time = 2.5 m/s * 1 s = 2.5 m
        let expected = &Iso2::new(Vec2::new(0.0, 2.5), na::zero());
        assert!(na::approx_eq_eps(rb_handle.borrow().position(), expected, &0.01),
                format!("Different impulse result is expected on body 1. Actual: {:?}, Expected: {:?}",
                        rb_handle.borrow().position(), expected));
        // expected result
        // impulse of 10 N*s on body with mass 8 (kg) results in
        // velocity = impulse/mass = 10 N*s / 8 kg = 1.25 m/s
        // distance = velocity * time = 1.25 m/s * 1 s = 1.25 m
        let expected = &Iso2::new(Vec2::new(5.0, 1.25), na::zero());
        assert!(na::approx_eq_eps(rb_handle2.borrow().position(), expected, &0.01),
                format!("Different impulse result is expected on body 2. Actual: {:?}, Expected: {:?}",
                        rb_handle2.borrow().position(), expected));


        // reset bodies
        rb_handle.borrow_mut().set_transformation(Iso2::new(na::zero(), na::zero()));
        rb_handle.borrow_mut().set_lin_vel(na::zero());
        rb_handle.borrow_mut().set_ang_vel(na::zero());
        rb_handle.borrow_mut().clear_forces();
        rb_handle.borrow_mut().activate(1.0);
        rb_handle2.borrow_mut().set_transformation(Iso2::new(Vec2::new(5.0, 0.0), na::zero()));
        rb_handle2.borrow_mut().set_lin_vel(na::zero());
        rb_handle2.borrow_mut().set_ang_vel(na::zero());
        rb_handle2.borrow_mut().clear_forces();
        rb_handle2.borrow_mut().activate(1.0);

        // torques have to work for different inertias
        rb_handle.borrow_mut().apply_angular_momentum(Vec1::new(10.0));
        rb_handle2.borrow_mut().apply_angular_momentum(Vec1::new(10.0));

        // simulate
        for _ in 0 .. 1000 {
            world.step(0.001);
        }

        // expected result
        // torque of 10 N*m*s on body with inertia of 2.67 kg*m^2 results in
        // rotation speed of rvel = 10 N*m*s / 2.67 kg*m^2 = 3.75 1/s
        // angle after 1s: 3.75
        let expected = &Iso2::new(Vec2::new(0.0, 0.0), Vec1::new(3.75));
        assert!(na::approx_eq_eps(rb_handle.borrow().position(), expected, &0.01),
                format!("Different torque result is expected on body 1. Actual: {:?}, Expected: {:?}",
                        rb_handle.borrow().position(), expected));
        // expected result
        // torque of 10 N*m*s on body with inertia of 5.33 kg*m^2 results in
        // rotation speed of rvel = 10 N*m*s / 5.33 kg*m^2 = 1.875 1/s
        // angle after 1s: 1.875
        let expected = &Iso2::new(Vec2::new(5.0, 0.0), Vec1::new(1.875));
        assert!(na::approx_eq_eps(rb_handle2.borrow().position(), expected, &0.01),
                format!("Different torque result is expected on body 2. Actual: {:?}, Expected: {:?}",
                        rb_handle2.borrow().position(), expected));


        // reset bodies
        rb_handle.borrow_mut().set_transformation(Iso2::new(na::zero(), na::zero()));
        rb_handle.borrow_mut().set_lin_vel(na::zero());
        rb_handle.borrow_mut().set_ang_vel(na::zero());
        rb_handle.borrow_mut().clear_forces();
        rb_handle.borrow_mut().activate(1.0);
        rb_handle2.borrow_mut().set_transformation(Iso2::new(Vec2::new(5.0, 0.0), na::zero()));
        rb_handle2.borrow_mut().set_lin_vel(na::zero());
        rb_handle2.borrow_mut().set_ang_vel(na::zero());
        rb_handle2.borrow_mut().clear_forces();
        rb_handle2.borrow_mut().activate(1.0);

        // nudge
        rb_handle.borrow_mut().apply_impulse_wrt_point(Vec2::new(0.0, 10.0), Vec2::new(1.0, 1.0));
        rb_handle2.borrow_mut().apply_impulse_wrt_point(Vec2::new(0.0, 10.0), Vec2::new(1.0, 1.0));

        // simulate
        for _ in 0 .. 1000 {
            world.step(0.001);
        }

        // expected values are the combination of the values in the two previous tests,
        // except with opposite rotation direction
        let expected = &Iso2::new(Vec2::new(0.0, 2.5), Vec1::new(3.75));
        assert!(na::approx_eq_eps(rb_handle.borrow().position(), expected, &0.01),
                format!("Different torque result is expected on body 1. Actual: {:?}, Expected: {:?}",
                        rb_handle.borrow().position(), expected));

        // expected values are the combination of the values in the two previous tests,
        // except with opposite rotation direction
        let expected = &Iso2::new(Vec2::new(5.0, 1.25), Vec1::new(1.875));
        assert!(na::approx_eq_eps(rb_handle2.borrow().position(), expected, &0.01),
                format!("Different torque result is expected on body 2. Actual: {:?}, Expected: {:?}",
                        rb_handle2.borrow().position(), expected));
    }
}
