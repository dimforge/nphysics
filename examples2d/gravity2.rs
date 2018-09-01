extern crate nalgebra as na;
extern crate ncollide2d;
extern crate nphysics2d;
extern crate nphysics_testbed2d;

use na::{Isometry2, Point3, Vector2};
use ncollide2d::shape::{Ball, Cuboid, ShapeHandle};
use nphysics2d::force_generator::ConstantAcceleration;
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

    // We setup two force generators that will replace the gravity.
    let mut up_gravity = ConstantAcceleration::new(Vector2::y() * -9.81, 0.0);
    let mut down_gravity = ConstantAcceleration::new(Vector2::y() * 9.81, 0.0);

    /*
     * Grouds
     */
    let ground_radx = 25.0;
    let ground_rady = 1.0;
    let ground_shape = ShapeHandle::new(Cuboid::new(Vector2::new(
        ground_radx - COLLIDER_MARGIN,
        ground_rady - COLLIDER_MARGIN,
    )));

    let ground_pos = Isometry2::new(-Vector2::y() * 2.0, na::zero());
    world.add_collider(
        COLLIDER_MARGIN,
        ground_shape.clone(),
        BodyPartHandle::ground(),
        ground_pos,
        Material::default(),
    );

    let ground_pos = Isometry2::new(Vector2::y() * 3.0, na::zero());
    world.add_collider(
        COLLIDER_MARGIN,
        ground_shape,
        BodyPartHandle::ground(),
        ground_pos,
        Material::default(),
    );

    /*
     * Create the balls
     */
    let num = 100f64 as usize;
    let rad = 0.2;
    let shift = 2.0 * rad;
    let centerx = shift * (num as f32) / 2.0;
    let centery = rad * 4.0;

    let geom = ShapeHandle::new(Ball::new(rad - COLLIDER_MARGIN));
    let inertia = geom.inertia(1.0);
    let center_of_mass = geom.center_of_mass();

    for i in 0usize..num {
        for j in 0usize..2 {
            let x = i as f32 * 2.5 * rad - centerx;
            let y = j as f32 * 2.5 * -rad + centery;

            /*
             * Create the rigid body.
             */
            let pos = Isometry2::new(Vector2::new(x, y), na::zero());
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

            /*
             * Set artifical gravity.
             */
            let color;

            if j == 1 {
                up_gravity.add_body_part(handle);
                color = Point3::new(0.0, 0.0, 1.0);
            } else {
                down_gravity.add_body_part(handle);
                color = Point3::new(0.0, 1.0, 0.0);
            }

            testbed.set_body_color(&world, handle, color);
        }
    }

    world.add_force_generator(up_gravity);
    world.add_force_generator(down_gravity);

    /*
     * Set up the testbed.
     */
    testbed.set_world(world);
    testbed.run();
}
