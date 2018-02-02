extern crate rand;
extern crate nalgebra as na;
extern crate ncollide;
extern crate nphysics2d;
extern crate nphysics_testbed2d;

use rand::random;
use na::{Vector2, Point2, Translation2};
use ncollide::shape::{Plane, ConvexHull};
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
     * First plane
     */
    let mut rb = RigidBody::new_static(Plane::new(Vector2::new(-1.0, -1.0)), 0.3, 0.6);

    rb.append_translation(&Translation2::new(0.0, 10.0));

    world.add_rigid_body(rb);

    /*
     * Second plane
     */
    let mut rb = RigidBody::new_static(Plane::new(Vector2::new(1.0, -1.0)), 0.3, 0.6);

    rb.append_translation(&Translation2::new(0.0, 10.0));

    world.add_rigid_body(rb);

    /*
     * Create the convex shapes
     */
    let npts = 10usize;
    let num = (1000.0f32.sqrt()) as usize;
    let rad = 1.0;
    let shift   = 2.5 * rad;
    let centerx = shift * (num as f32) / 2.0;
    let centery = shift * (num as f32) / 2.0;

    for i in 0usize .. num {
        for j in 0usize .. num {
            let x = i as f32 * 2.5 * rad - centerx;
            let y = j as f32 * 2.5 * rad - centery * 2.0 - 25.0;

            let mut pts = Vec::with_capacity(npts);

            for _ in 0 .. npts {
                pts.push(random::<Point2<f32>>() * 2.0 + Vector2::new(5.0, 5.0));
            }

            let geom = ConvexHull::new(pts);
            let mut rb = RigidBody::new_dynamic(geom, 0.1, 0.3, 0.6);
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
