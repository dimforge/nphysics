extern crate nalgebra as na;
extern crate ncollide2d;
extern crate nphysics2d;
extern crate nphysics_testbed2d;

use na::{Point2, Vector2};
use ncollide2d::shape::{Compound, Cuboid, ShapeHandle};
use nphysics2d::object::{ColliderDesc, RigidBodyDesc};
use nphysics2d::world::World;
use nphysics_testbed2d::Testbed;


pub fn init_world(testbed: &mut Testbed) {
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
        .translation(-Vector2::y())
        .build(&mut world);

    /*
     * Cross shaped geometry
     */
    let mut cross_geoms = Vec::new();

    let large_rad = 1.0f32;
    let small_rad = 0.05f32;

    let edge_x = Cuboid::new(Vector2::new(large_rad, small_rad));
    let edge_y = Cuboid::new(Vector2::new(small_rad, large_rad));

    cross_geoms.push((na::one(), ShapeHandle::new(edge_x)));
    cross_geoms.push((na::one(), ShapeHandle::new(edge_y)));

    let compound = Compound::new(cross_geoms);
    let cross = ShapeHandle::new(compound);
    let collider_desc = ColliderDesc::new(cross)
        .density(1.0);

    /*
     * Create the boxes
     */
    let num = 15;
    let shift = 2.5 * large_rad;
    let centerx = shift * (num as f32) / 2.0;
    let centery = shift * (num as f32) / 2.0;

    let mut rb_desc = RigidBodyDesc::new()
        .collider(&collider_desc);

    for i in 0usize..num {
        for j in 0usize..num {
            let x = i as f32 * 2.5 * large_rad - centerx;
            let y = j as f32 * 2.5 * -large_rad + centery * 2.0;

            // Build the rigid body and its collider.
            rb_desc
                .set_translation(Vector2::new(x, y))
                .build(&mut world);
        }
    }

    /*
     * Run the simulation.
     */
    testbed.set_world(world);
    testbed.look_at(Point2::new(0.0, -8.0), 30.0);
}


fn main() {
    let mut testbed = Testbed::new_empty();
    init_world(&mut testbed);
    testbed.run();
}