extern crate nalgebra as na;
extern crate ncollide2d;
extern crate nphysics2d;
extern crate nphysics_testbed2d;

use na::{Point2, Vector2};
use ncollide2d::shape::{Cuboid, Capsule, ShapeHandle};
use nphysics2d::object::{ColliderDesc, RigidBodyDesc};
use nphysics2d::world::World;
use nphysics_testbed2d::Testbed;

const COLLIDER_MARGIN: f32 = 0.01;

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
     * Create the boxes
     */
    let num = 25;
    let rad = 0.1;
    let half_height = 0.1;

    let capsule = ShapeHandle::new(Capsule::new(
        half_height,
        rad - COLLIDER_MARGIN,
    ));

    let collider_desc = ColliderDesc::new(capsule)
        .with_density(1.0);

    let mut rb_desc = RigidBodyDesc::default()
        .with_collider(&collider_desc);

    let shiftx = (rad + collider_desc.margin()) * 2.0;
    let shifty = (half_height + rad) * 2.0;
    let centerx = shiftx * (num as f32) / 2.0;
    let centery = shifty / 2.0;

    for i in 0usize..num {
        for j in 0..num {
            let x = i as f32 * shiftx - centerx;
            let y = j as f32 * shifty + centery;

            // Build the rigid body and its collider.
            rb_desc
                .set_translation(Vector2::new(x, y))
                .build(&mut world);
        }
    }

    /*
     * Set up the testbed.
     */
    let mut testbed = Testbed::new(world);
    testbed.look_at(Point2::new(0.0, -2.5), 95.0);
    testbed.run();
}
