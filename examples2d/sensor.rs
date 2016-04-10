extern crate nalgebra as na;
extern crate ncollide;
extern crate nphysics2d;
extern crate nphysics_testbed2d;

use na::{Vec2, Pnt3, Translation};
use ncollide::shape::{Plane, Cuboid, Ball};
use ncollide::geometry::Proximity;
use ncollide::narrow_phase::ProximitySignalHandler;
use nphysics2d::world::World;
use nphysics2d::object::{RigidBody, WorldObject};
use nphysics2d::object::Sensor;
use nphysics_testbed2d::{Testbed, GraphicsManagerHandle};

/*
 * A proximity handler that will change the color of a rigid body as it enters of leaves proximity
 * with the sensor.
 */
struct ColorChanger {
    graphics: GraphicsManagerHandle
}

impl ProximitySignalHandler<WorldObject<f32>> for ColorChanger {
    fn handle_proximity(&mut self,
                        o1: &WorldObject<f32>, o2: &WorldObject<f32>,
                        _: Proximity, new_proximity: Proximity) {
        let color = match new_proximity {
            Proximity::WithinMargin | Proximity::Intersecting => Pnt3::new(1.0, 1.0, 0.0),
            Proximity::Disjoint                               => Pnt3::new(0.5, 0.5, 1.0)
        };
    
        if let WorldObject::RigidBody(ref rb) = *o1 {
            self.graphics.borrow_mut().set_rigid_body_color(rb, color);
        }
    
        if let WorldObject::RigidBody(ref rb) = *o2 {
            self.graphics.borrow_mut().set_rigid_body_color(rb, color);
        }
    }
}

fn main() {
    let mut testbed = Testbed::new_empty();

    /*
     * World
     */
    let mut world = World::new();
    world.set_gravity(Vec2::new(0.0, 9.81));

    /*
     * Plane
     */
    let geom = Plane::new(Vec2::new(0.0, -1.0));
    world.add_rigid_body(RigidBody::new_static(geom, 0.2, 0.6));

    /*
     * Create some boxes.
     */
    let num     = 15;
    let rad     = 1.0;
    let shift   = rad * 2.0;
    let centerx = shift * (num as f32) / 2.0;

    for i in 0usize .. num {
        let x = i as f32 * shift - centerx;

        let geom   = Cuboid::new(Vec2::new(rad - 0.04, rad - 0.04));
        let mut rb = RigidBody::new_dynamic(geom, 1.0, 0.2, 0.5);

        rb.append_translation(&Vec2::new(x, -1.0));

        let rb_handle = world.add_rigid_body(rb);
        testbed.set_rigid_body_color(&rb_handle, Pnt3::new(0.5, 0.5, 1.0));
    }

    /*
     * Create a box that will have a sensor attached.
     */
    let geom   = Cuboid::new(Vec2::new(0.5f32, 0.5));
    let mut rb = RigidBody::new_dynamic(geom, 1.0, 0.2, 0.5);
    rb.append_translation(&Vec2::new(0.0, -10.0));
    let rb_handle = world.add_rigid_body(rb);
    testbed.set_rigid_body_color(&rb_handle, Pnt3::new(0.5, 1.0, 1.0));

    // Attach a sensor.
    let sensor_geom = Ball::new(rad * 5.0);
    let sensor      = Sensor::new(sensor_geom, Some(rb_handle));
    world.add_sensor(sensor);

    // Setup the callback.
    let handler = ColorChanger { graphics: testbed.graphics() };
    world.register_proximity_signal_handler("color_changer", handler);

    /*
     * Set up the testbed.
     */
    testbed.set_world(world);
    testbed.run();
}
