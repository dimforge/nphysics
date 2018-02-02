extern crate nalgebra as na;
extern crate ncollide;
extern crate nphysics2d;
extern crate nphysics_testbed2d;

use std::f32::consts::PI;
use na::{Point2, Point3, Vector2, Translation2, Isometry2};
use ncollide::query::Proximity;
use ncollide::shape::{ShapeHandle, Plane, Cuboid, Ball};
use nphysics2d::world::World;
use nphysics2d::object::BodyHandle;
use nphysics2d::joint::FreeJoint;
use nphysics2d::volumetric::Volumetric;
use nphysics_testbed2d::Testbed;

const COLLIDER_MARGIN: f32 = 0.01;

fn main() {
    let mut testbed = Testbed::new_empty();

    /*
     * World
     */
    let mut world = World::new();
    world.set_gravity(Vector2::new(0.0, 9.81));

    /*
     * Ground.
     */
    let ground_shape = ShapeHandle::new(Cuboid::new(Vector2::new(10.0, 10.0)));
    let ground_pos   = Isometry2::new(Vector2::y() * 10.0, na::zero());
    world.add_collider(COLLIDER_MARGIN, ground_shape, BodyHandle::ground(), ground_pos, false);

    /*
     * Create some boxes.
     */
    let num     = 15;
    let rad     = 0.2;
    let shift   = rad * 2.0;
    let centerx = shift * (num as f32) / 2.0;
    let centerz = shift * (num as f32) / 2.0;

    let geom    = ShapeHandle::new(Cuboid::new(Vector2::repeat(rad - COLLIDER_MARGIN)));
    let inertia = geom.inertia(1.0);

    for i in 0usize .. num {
        let x = i as f32 * shift - centerx;

        /*
         * Create the rigid body.
         */
        let pos    = Isometry2::new(Vector2::new(x, -2.0), na::zero());
        let handle = world.add_rigid_body(pos, inertia);

        /*
         * Create the collider.
         */
        world.add_collider(COLLIDER_MARGIN, geom.clone(), handle, Isometry2::identity(), true);
        testbed.set_body_color(&world, handle, Point3::new(0.5, 0.5, 1.0));
    }

    /*
     * Create a box that will have a sensor attached.
     */
    let pos         = Isometry2::new(Vector2::new(0.0, -10.0), na::zero());
    let sensor_body = world.add_rigid_body(pos, inertia);
    let sensor_geom = ShapeHandle::new(Ball::new(rad * 5.0));

    world.add_collider(COLLIDER_MARGIN, geom, sensor_body, Isometry2::identity(), true);
    world.add_sensor(sensor_geom, sensor_body, Isometry2::identity(), true);
    testbed.set_body_color(&world, sensor_body, Point3::new(0.5, 1.0, 1.0));

    // Callback that will be executed on the main loop to handle proximities.
    let graphics = testbed.graphics();
    testbed.add_callback(move |world, _| {
        for prox in world.proximity_events() {
            let color = match prox.new_status {
                Proximity::WithinMargin | Proximity::Intersecting => Point3::new(1.0, 1.0, 0.0),
                Proximity::Disjoint                               => Point3::new(0.5, 0.5, 1.0)
            };

            let body1 = world.collider(prox.collider1).data.body;
            let body2 = world.collider(prox.collider2).data.body;

            if !body1.is_ground() && body1 != sensor_body {
                graphics.borrow_mut().set_body_color(world, body1, color);
            }
            if !body2.is_ground() && body2 != sensor_body {
                graphics.borrow_mut().set_body_color(world, body2, color);
            }
        }
    });

    /*
     * Set up the testbed.
     */
    testbed.set_world(world);
    testbed.run();
}
