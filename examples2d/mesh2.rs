extern crate alga;
extern crate nalgebra as na;
extern crate ncollide;
extern crate nphysics2d;
extern crate nphysics_testbed2d;
extern crate rand;

use na::Real;
use std::sync::Arc;
use rand::{Rng, SeedableRng, StdRng};
use na::{Isometry2, Point2, Vector2};
use ncollide::shape::{Cuboid, Polyline, ShapeHandle};
use nphysics2d::volumetric::Volumetric;
use nphysics2d::world::World;
use nphysics2d::object::{BodyHandle, Material};
use nphysics_testbed2d::Testbed;

const COLLIDER_MARGIN: f32 = 0.01;

fn main() {
    /*
     * World
     */
    let mut world = World::new();
    world.set_gravity(Vector2::new(0.0, 9.81));

    /*
     * Polyline
     */
    let num_split = 20;
    let begin = -15.0;
    let max_h = 3.0;
    let begin_h = 3.0;
    let step = (begin.abs() * 2.0) / (num_split as f32);
    let mut vertices: Vec<Point2<f32>> = (0..num_split + 2)
        .map(|i| Point2::new(begin + (i as f32) * step, 0.0))
        .collect();
    let mut indices = Vec::new();
    let mut rng: StdRng = SeedableRng::from_seed(&[1, 2, 3, 4][..]);

    for i in 0usize..num_split {
        let h: f32 = rng.gen();
        vertices[i + 1].y = begin_h - h * max_h;

        indices.push(Point2::new(i, i + 1));
    }
    indices.push(Point2::new(num_split, num_split + 1));

    let mesh = Polyline::new(Arc::new(vertices), Arc::new(indices), None, None);
    world.add_collider(
        COLLIDER_MARGIN,
        ShapeHandle::new(mesh),
        BodyHandle::ground(),
        Isometry2::identity(),
        Material::default(),
    );

    /*
     * Create the boxes
     */
    let width = 100;
    let height = 10;
    let rad = 0.1;
    let shift = 2.0 * rad;
    let centerx = shift * (width as f32) / 2.0;

    let geom = ShapeHandle::new(Cuboid::new(Vector2::new(
        rad - COLLIDER_MARGIN,
        rad - COLLIDER_MARGIN,
    )));
    let inertia = geom.inertia(1.0);
    let center_of_mass = geom.center_of_mass();

    for i in 0usize..height {
        for j in 0usize..width {
            let fj = j as f32;
            let fi = i as f32;
            let x = fj * 2.0 * rad - centerx;
            let y = -fi * 2.0 * rad - 1.0;

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
        }
    }

    /*
     * Run the simulation.
     */
    let mut testbed = Testbed::new(world);

    testbed.run();
}
