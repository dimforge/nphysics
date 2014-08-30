extern crate native;
extern crate nalgebra;
extern crate ncollide = "ncollide3df32";
extern crate nphysics = "nphysics3df32";
extern crate nphysics_testbed3d;

use std::rc::Rc;
use std::cell::RefCell;
use std::rand;
use nalgebra::na::{Vec3, Translation};
use ncollide::geom::{Plane, Convex};
use nphysics::world::World;
use nphysics::object::RigidBody;
use nphysics_testbed3d::Testbed;

#[start]
fn start(argc: int, argv: *const *const u8) -> int {
    native::start(argc, argv, main)
}

fn main() {
    /*
     * World
     */
    let mut world = World::new();
    world.set_gravity(Vec3::new(0.0f32, -9.81, 0.0));

    /*
     * Plane
     */
    let geom = Plane::new(Vec3::new(0.0f32, 1.0, 0.0));
    let body = Rc::new(RefCell::new(RigidBody::new_static(geom, 0.3, 0.6)));

    world.add_body(body.clone());

    /*
     * Create the convex geometries.
     */
    let npts    = 10u;
    let num     = 8;
    let shift   = 2.0;
    let centerx = shift * (num as f32) / 2.0;
    let centery = shift / 2.0;
    let centerz = shift * (num as f32) / 2.0;

    for i in range(0u, num) {
        for j in range(0u, num) {
            for k in range(0u, num) {
                let x = i as f32 * shift - centerx;
                let y = j as f32 * shift + centery;
                let z = k as f32 * shift - centerz;

                let mut pts = Vec::with_capacity(npts);

                for _ in range(0, npts) {
                    pts.push(rand::random::<Vec3<f32>>() * 2.0f32 + Vec3::new(5.0f32, 5.0f32, 5.0f32));
                }

                let geom = Convex::new(pts.as_slice());
                let mut rb = RigidBody::new_dynamic(geom, 1.0f32, 0.3, 0.5);

                rb.append_translation(&Vec3::new(x, y, z));

                let body = Rc::new(RefCell::new(rb));

                world.add_body(body.clone());
            }
        }
    }

    /*
     * Set up the testbed.
     */
    let mut testbed = Testbed::new(world);

    testbed.look_at(Vec3::new(-30.0, 30.0, -30.0), Vec3::new(0.0, 0.0, 0.0));
    testbed.run();
}
