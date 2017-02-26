extern crate rand;
extern crate alga;
extern crate nalgebra as na;
extern crate ncollide;
extern crate nphysics2d;
extern crate nphysics_testbed2d;

use alga::general::Real;
use std::sync::Arc;
use rand::{StdRng, SeedableRng, Rng};
use na::{Point2, Vector2, Translation2};
use ncollide::shape::{Cuboid, Polyline};
use nphysics2d::world::World;
use nphysics2d::object::RigidBody;
use nphysics_testbed2d::Testbed;

fn main() {
    /*
     * World
     */
    let mut world = World::new();
    world.set_gravity(Vector2::new(0.0, 9.81));

    /*
     * Polyline
     */
    let num_split = 5;
    let begin     = -75.0;
    let max_h     = 15.0;
    let begin_h   = 15.0;
    let step      = (begin.abs() * 2.0) / (num_split as f32);
    let mut vertices: Vec<Point2<f32>> = (0 .. num_split + 2).map(|i| Point2::new(begin + (i as f32) * step, 0.0)).collect();
    let mut indices  = Vec::new();
    let mut rng: StdRng = SeedableRng::from_seed(&[1, 2, 3, 4][..]);

    for i in 0usize .. num_split {
        let h: f32 = rng.gen();
        vertices[i + 1].y = begin_h - h * max_h;

        indices.push(Point2::new(i, i + 1));
    }
    indices.push(Point2::new(num_split, num_split + 1));

    let mesh = Polyline::new(Arc::new(vertices), Arc::new(indices), None, None);
    let rb = RigidBody::new_static(mesh, 0.3, 0.6);

    world.add_rigid_body(rb);

    /*
     * Create the boxes
     */
    let width   = 100;
    let height  = 20;
    let rad     = 0.5;
    let shift   = 2.0 * rad;
    let centerx = shift * (width as f32) / 2.0;

    for i in 0usize .. height {
        for j in 0usize .. width {
            let fj = j as f32;
            let fi = i as f32;
            let x = fj * 2.0 * rad - centerx;
            let y = -fi * 2.0 * rad;

            let mut rb = RigidBody::new_dynamic(Cuboid::new(Vector2::new(rad, rad)), 1.0, 0.3, 0.6);

            rb.append_translation(&Translation2::new(x, y));

            world.add_rigid_body(rb);
        }
    }

    /*
     * Run the simulation.
     */
    let mut testbed = Testbed::new(world);

    testbed.run();
}
