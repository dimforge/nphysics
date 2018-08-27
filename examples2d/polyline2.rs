extern crate env_logger;
extern crate nalgebra as na;
extern crate ncollide2d;
extern crate nphysics2d;
extern crate nphysics_testbed2d;
extern crate rand;

use na::Real;
use na::{Isometry2, Point2, Vector2};
use ncollide2d::shape::{Cuboid, Polyline, ShapeHandle};
use nphysics2d::object::{BodyHandle, Material};
use nphysics2d::volumetric::Volumetric;
use nphysics2d::world::World;
use nphysics_testbed2d::Testbed;
use rand::{Rng, XorShiftRng};

const COLLIDER_MARGIN: f32 = 0.01;

fn main() {
    env_logger::init();
    /*
     * World
     */
    let mut world = World::new();
    world.set_gravity(Vector2::new(0.0, -9.81));

    /*
     * Polyline
     */
    let num_split = 20;
    let begin = -15.0;
    let max_h = 3.0;
    let begin_h = -3.0;
    let step = (begin.abs() * 2.0) / (num_split as f32);
    let mut vertices: Vec<Point2<f32>> = (0..num_split + 2)
        .map(|i| Point2::new(begin + (i as f32) * step, 0.0))
        .collect();
    let mut rng = XorShiftRng::new_unseeded();

    for i in 0usize..num_split {
        let h: f32 = rng.gen();
        vertices[i + 1].y = h * max_h + begin_h;
    }

    let polyline = Polyline::new(vertices);
    world.add_collider(
        COLLIDER_MARGIN,
        ShapeHandle::new(polyline),
        BodyHandle::ground(),
        Isometry2::identity(),
        Material::default(),
    );

    /*
     * Create the boxes
     */
    let width = 75;
    let height = 7;
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
            let y = fi * 2.0 * rad + 1.0;

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
    testbed.look_at(Point2::origin(), 75.0);
    testbed.run();
}
