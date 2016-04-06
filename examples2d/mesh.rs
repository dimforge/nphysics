extern crate num;
extern crate rand;
extern crate nalgebra as na;
extern crate ncollide;
extern crate nphysics2d;
extern crate nphysics_testbed2d;

use std::sync::Arc;
use num::Float;
use rand::{StdRng, SeedableRng, Rng};
use na::{Pnt2, Vec2, Translation};
use ncollide::shape::{Cuboid, Polyline};
use nphysics2d::world::World;
use nphysics2d::object::RigidBody;
use nphysics_testbed2d::Testbed;

fn main() {
    /*
     * World
     */
    let mut world = World::new();
    world.set_gravity(Vec2::new(0.0, 9.81));

    /*
     * Polyline
     */
    let num_split = 5;
    let begin     = -75.0;
    let max_h     = 15.0;
    let begin_h   = 15.0;
    let step      = (begin.abs() * 2.0) / (num_split as f32);
    let mut vertices: Vec<Pnt2<f32>> = (0 .. num_split + 2).map(|i| Pnt2::new(begin + (i as f32) * step, 0.0)).collect();
    let mut indices  = Vec::new();
    let mut rng: StdRng = SeedableRng::from_seed(&[1, 2, 3, 4][..]);

    for i in 0usize .. num_split {
        let h: f32 = rng.gen();
        vertices[i + 1].y = begin_h - h * max_h;

        indices.push(Pnt2::new(i, i + 1));
    }
    indices.push(Pnt2::new(num_split, num_split + 1));

    let mesh = Polyline::new(Arc::new(vertices), Arc::new(indices), None, None);
    let rb = RigidBody::new_static(mesh, 0.3, 0.6, None);

    world.add_body(rb);

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

            let mut rb = RigidBody::new_dynamic(Cuboid::new(Vec2::new(rad, rad)), 1.0, 0.3, 0.6, None);

            rb.append_translation(&Vec2::new(x, y));

            world.add_body(rb);
        }
    }

    /*
     * Run the simulation.
     */
    let mut testbed = Testbed::new(world);

    testbed.run();
}
