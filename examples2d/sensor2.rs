extern crate nalgebra as na;
extern crate ncollide2d;
extern crate nphysics2d;
extern crate nphysics_testbed2d;

use na::{Isometry2, Point2, Point3, Vector2};
use ncollide2d::query::Proximity;
use ncollide2d::shape::{Ball, Cuboid, ShapeHandle};
use nphysics2d::object::{BodyPartHandle, Material};
use nphysics2d::volumetric::Volumetric;
use nphysics2d::world::World;
use nphysics_testbed2d::Testbed;

const COLLIDER_MARGIN: f32 = 0.01;

fn main() {
    let mut testbed = Testbed::new_empty();

    /*
     * World
     */
    let mut world = World::new();
    world.set_gravity(Vector2::new(0.0, -9.81));

    /*
     * Ground.
     */
    let ground_shape = ShapeHandle::new(Cuboid::new(Vector2::new(10.0, 1.0)));
    let ground_pos = Isometry2::new(-Vector2::y(), na::zero());
    world.add_collider(
        COLLIDER_MARGIN,
        ground_shape,
        BodyPartHandle::ground(),
        ground_pos,
        Material::default(),
    );

    /*
     * Create some boxes.
     */
    let num = 15;
    let rad = 0.2;
    let shift = rad * 2.0;
    let centerx = shift * (num as f32) / 2.0;

    let geom = ShapeHandle::new(Cuboid::new(Vector2::repeat(rad - COLLIDER_MARGIN)));
    let inertia = geom.inertia(1.0);
    let center_of_mass = geom.center_of_mass();

    for i in 0usize..num {
        let x = i as f32 * shift - centerx;

        /*
         * Create the rigid body.
         */
        let pos = Isometry2::new(Vector2::new(x, 2.0), na::zero());
        let handle = world.add_rigid_body(pos, inertia, center_of_mass);

        /*
         * Create the collider.
         */
        world.add_collider(
            COLLIDER_MARGIN,
            geom.clone(),
            handle,
            Isometry2::identity(),
            Material::default(),
        );
        testbed.set_body_color(handle.0, Point3::new(0.5, 0.5, 1.0));
    }

    /*
     * Create a box that will have a sensor attached.
     */
    let pos = Isometry2::new(Vector2::new(0.0, 4.0), na::zero());
    let sensor_body = world.add_rigid_body(pos, inertia, center_of_mass);
    let sensor_geom = ShapeHandle::new(Ball::new(rad * 5.0));

    world.add_collider(
        COLLIDER_MARGIN,
        geom,
        sensor_body,
        Isometry2::identity(),
        Material::default(),
    );
    world.add_sensor(sensor_geom, sensor_body, Isometry2::identity());
    testbed.set_body_color(sensor_body.0, Point3::new(0.5, 1.0, 1.0));

    // Callback that will be executed on the main loop to handle proximities.
    testbed.add_callback(move |world, graphics, _| {
        for prox in world.proximity_events() {
            let color = match prox.new_status {
                Proximity::WithinMargin | Proximity::Intersecting => Point3::new(1.0, 1.0, 0.0),
                Proximity::Disjoint => Point3::new(0.5, 0.5, 1.0),
            };

            let body1 = world.collider(prox.collider1).unwrap().body();
            let body2 = world.collider(prox.collider2).unwrap().body();

            if !body1.is_ground() && body1 != sensor_body.0 {
                graphics.set_body_color(body1, color);
            }
            if !body2.is_ground() && body2 != sensor_body.0 {
                graphics.set_body_color(body2, color);
            }
        }
    });

    /*
     * Set up the testbed.
     */
    testbed.set_world(world);
    testbed.look_at(Point2::origin(), 75.0);
    testbed.run();
}
