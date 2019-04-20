extern crate nalgebra as na;
extern crate ncollide2d;
extern crate nphysics2d;
extern crate nphysics_testbed2d;
extern crate rand;

use na::{Point2, Vector2, DVector};
use ncollide2d::shape::{Cuboid, HeightField, ShapeHandle};
use nphysics2d::object::{RigidBodyDesc, ColliderDesc};
use nphysics2d::world::World;
use nphysics_testbed2d::Testbed;
use rand::{Rng, SeedableRng, StdRng};


pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut world = World::new();
    world.set_gravity(Vector2::new(0.0, -9.81));

    /*
     * Polyline
     */
    let mut rng: StdRng = SeedableRng::from_seed([0; 32]);
    let heights = DVector::from_fn(20, |_, _| rng.gen::<f32>());

    let mut heightfield = HeightField::new(heights, Vector2::new(20.0, 1.0));

    // It is possible to remove some segments from the heightfield.
    heightfield.set_segment_removed(3, true);
    heightfield.set_segment_removed(13, true);

    ColliderDesc::new(ShapeHandle::new(heightfield))
        .build(&mut world);

    /*
     * Create the boxes
     */
    let width = 75;
    let height = 7;
    let rad = 0.1;

    let cuboid = ShapeHandle::new(Cuboid::new(Vector2::repeat(rad)));
    let collider_desc = ColliderDesc::new(cuboid)
        .density(1.0);

    let mut rb_desc = RigidBodyDesc::new()
        .collider(&collider_desc);

    let shift = 2.0 * (rad + collider_desc.get_margin());
    let centerx = shift * (width as f32) / 2.0;

    for i in 0usize..height {
        for j in 0usize..width {
            let fj = j as f32;
            let fi = i as f32;
            let x = fj * shift - centerx;
            let y = fi * shift + 1.0;

            // Create the rigid body and its collider.
            rb_desc
                .set_translation(Vector2::new(x, y))
                .build(&mut world);
        }
    }

    /*
     * Run the simulation.
     */
    testbed.set_world(world);
    testbed.look_at(Point2::origin(), 75.0);
}


fn main() {
    let mut testbed = Testbed::new_empty();
    init_world(&mut testbed);
    testbed.run();
}