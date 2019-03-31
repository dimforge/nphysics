extern crate nalgebra as na;
extern crate ncollide2d;
extern crate nphysics2d;
extern crate nphysics_testbed2d;
extern crate rand;

use na::{Point2, Vector2};
use ncollide2d::shape::{Cuboid, Polyline, ShapeHandle};
use nphysics2d::object::{ColliderDesc, RigidBodyDesc};
use nphysics2d::world::World;
use nphysics_testbed2d::Testbed;

use rand::distributions::{Standard, Distribution};
use rand::{SeedableRng, XorShiftRng};


fn main() {
    /*
     * World
     */
    let mut world = World::new();
    world.set_gravity(Vector2::new(0.0, -9.81));

    /*
     * Polyline
     */
    let num_split = 20;
    let begin = -15.0f32;
    let max_h = 3.0;
    let begin_h = -3.0;
    let step = (begin.abs() * 2.0) / (num_split as f32);
    let mut vertices: Vec<Point2<f32>> = (0..num_split + 2)
        .map(|i| Point2::new(begin + (i as f32) * step, 0.0))
        .collect();

    let mut rng = XorShiftRng::seed_from_u64(42);
    let distribution = Standard;

    for i in 0usize..num_split {
        let h: f32 = distribution.sample(&mut rng);
        vertices[i + 1].y = h * max_h + begin_h;
    }

    let polyline = ShapeHandle::new(Polyline::new(vertices, None));
    ColliderDesc::new(polyline).build(&mut world);

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
            let y = fi * shift + 0.5;

            // Create the rigid body and its collider.
            rb_desc
                .set_translation(Vector2::new(x, y))
                .build(&mut world);
        }
    }

    /*
     * Run the simulation.
     */
    let mut testbed = Testbed::new(world);
    testbed.look_at(Point2::origin(), 75.0);
    testbed.run();
}
