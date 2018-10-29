extern crate nalgebra as na;
extern crate ncollide3d;
extern crate nphysics3d;
extern crate nphysics_testbed3d;

use na::{Isometry3, Point3, Vector3};
use ncollide3d::shape::{Ball, Plane, ShapeHandle};
use nphysics3d::force_generator::ConstantAcceleration;
use nphysics3d::object::{BodyHandle, Material};
use nphysics3d::volumetric::Volumetric;
use nphysics3d::world::World;
use nphysics_testbed3d::Testbed;

const COLLIDER_MARGIN: f32 = 0.01;

fn main() {
    let mut testbed = Testbed::new_empty();

    /*
     * World
     */
    let mut world = World::new();

    // We setup two force generators that will replace the gravity.
    let mut up_gravity = ConstantAcceleration::new(Vector3::y() * 9.81, Vector3::zeros());
    let mut down_gravity = ConstantAcceleration::new(Vector3::y() * -9.81, Vector3::zeros());

    /*
     * Planes
     */
    let ground_shape = ShapeHandle::new(Plane::new(Vector3::y_axis()));
    world.add_collider(
        COLLIDER_MARGIN,
        ground_shape,
        BodyHandle::ground(),
        Isometry3::identity(),
        Material::default(),
    );

    let ground_shape = ShapeHandle::new(Plane::new(-Vector3::y_axis()));
    world.add_collider(
        COLLIDER_MARGIN,
        ground_shape,
        BodyHandle::ground(),
        Isometry3::new(Vector3::y() * 20.0, na::zero()),
        Material::default(),
    );

    /*
     * Create the balls
     */
    let num = 1000f64.sqrt() as usize;
    let rad = 0.1;
    let shift = 0.25 * rad;
    let centerx = shift * (num as f32) / 2.0;
    let centery = 0.5;

    let geom = ShapeHandle::new(Ball::new(rad - COLLIDER_MARGIN));
    let inertia = geom.inertia(1.0);
    let center_of_mass = geom.center_of_mass();

    for i in 0usize..num {
        for j in 0usize..2 {
            for k in 0usize..num {
                let x = i as f32 * 2.5 * rad - centerx;
                let y = 1.0 + j as f32 * 2.5 * rad + centery;
                let z = k as f32 * 2.5 * rad - centerx;

                /*
                 * Create the rigid body.
                 */
                let pos = Isometry3::new(Vector3::new(x, y, z), na::zero());
                let handle = world.add_rigid_body(pos, inertia, center_of_mass);

                /*
                 * Create the collider.
                 */
                world.add_collider(
                    COLLIDER_MARGIN,
                    geom.clone(),
                    handle,
                    Isometry3::identity(),
                    Material::default(),
                );

                /*
                 * Set artifical gravity.
                 */
                let color = if j == 1 {
                    up_gravity.add_body_part(handle);
                    Point3::new(0.0, 0.0, 1.0)
                } else {
                    down_gravity.add_body_part(handle);
                    Point3::new(0.0, 1.0, 0.0)
                };

                testbed.set_body_color(&world, handle, color);
            }
        }
    }

    world.add_force_generator(up_gravity);
    world.add_force_generator(down_gravity);

    /*
     * Set up the testbed.
     */
    testbed.set_world(world);
    testbed.look_at(Point3::new(-1.0, 5.0, -1.0), Point3::new(0.0, 0.0, 0.0));
    testbed.run();
}
