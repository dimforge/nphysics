extern crate nalgebra as na;
extern crate ncollide2d;
extern crate nphysics2d;
extern crate nphysics_testbed2d;

use na::{Isometry2, Vector2};
use ncollide2d::shape::{Compound, Cuboid, ShapeHandle};
use nphysics2d::object::{ColliderDesc, RigidBodyDesc};
use nphysics2d::world::World;
use nphysics_testbed2d::Testbed;


fn main() {
    /*
     * World
     */
    let mut world = World::new();
    world.set_gravity(Vector2::new(0.0, -9.81));

    /*
     * Ground
     */
    let ground_size = 25.0;
    let ground_shape =
        ShapeHandle::new(Cuboid::new(Vector2::new(ground_size, 1.0)));

    ColliderDesc::new(ground_shape)
        .with_translation(-Vector2::y())
        .build(&mut world);

    /*
     * Cross shaped geometry
     */
    let large_rad = 1.0f32;
    let small_rad = 0.05f32;

    let delta1 = Isometry2::new(Vector2::new(0.0, large_rad), na::zero());
    let delta2 = Isometry2::new(Vector2::new(-large_rad, 0.0), na::zero());
    let delta3 = Isometry2::new(Vector2::new(large_rad, 0.0), na::zero());

    let mut cross_geoms = Vec::new();
    let vertical = ShapeHandle::new(Cuboid::new(Vector2::new(
        small_rad,
        large_rad,
    )));
    let horizontal = ShapeHandle::new(Cuboid::new(Vector2::new(
        large_rad,
        small_rad,
    )));
    cross_geoms.push((delta1, horizontal));
    cross_geoms.push((delta2, vertical.clone()));
    cross_geoms.push((delta3, vertical));

    let compound = Compound::new(cross_geoms);
    let cross = ShapeHandle::new(compound);

    let collider_desc = ColliderDesc::new(cross)
        .with_density(1.0);


    /*
     * Create the rigid bodies.
     */
    let num = 15;
    let shift = 2.5 * large_rad;
    let centerx = (shift + collider_desc.margin()) * (num as f32) / 2.0;
    let centery = (shift + collider_desc.margin()) * (num as f32) / 2.0;

    let mut rb_desc = RigidBodyDesc::default()
        .with_collider(&collider_desc);

    for i in 0usize..num {
        for j in 0usize..num {
            let x = i as f32 * 2.5 * large_rad - centerx;
            let y = j as f32 * 2.5 * -large_rad + centery * 2.0;

            // Build the rigid body and its collider.
            let _ = rb_desc
                .set_translation(Vector2::new(x, y))
                .build(&mut world);
        }
    }

    /*
     * Run the simulation.
     */
    let testbed = Testbed::new(world);
    testbed.run();
}
