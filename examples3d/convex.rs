extern crate rand;
extern crate nalgebra as na;
extern crate ncollide;
extern crate nphysics3d;
extern crate nphysics_testbed3d;

use rand::random;
use na::{Pnt3, Vec3, Translation};
use ncollide::shape::{Plane, Convex};
use nphysics3d::world::World;
use nphysics3d::object::RigidBody;
use nphysics_testbed3d::Testbed;

fn main() {
    /*
     * World
     */
    let mut world = World::new();
    world.set_gravity(Vec3::new(0.0, -9.81, 0.0));

    /*
     * Plane
     */
    let geom = Plane::new(Vec3::new(0.0, 1.0, 0.0));

    world.add_body(RigidBody::new_static(geom, 0.3, 0.6, None));

    /*
     * Create the convex geometries.
     */
    let npts    = 10usize;
    let num     = 8;
    let shift   = 2.0;
    let centerx = shift * (num as f32) / 2.0;
    let centery = shift / 2.0;
    let centerz = shift * (num as f32) / 2.0;

    for i in 0usize .. num {
        for j in 0usize .. num {
            for k in 0usize .. num {
                let x = i as f32 * shift - centerx;
                let y = j as f32 * shift + centery;
                let z = k as f32 * shift - centerz;

                let mut pts = Vec::with_capacity(npts);

                for _ in 0 .. npts {
                    pts.push(random::<Pnt3<f32>>() * 2.0+ Vec3::new(5.0, 5.0, 5.0));
                }

                let geom = Convex::new(pts);
                let mut rb = RigidBody::new_dynamic(geom, 1.0, 0.3, 0.5, None);

                rb.append_translation(&Vec3::new(x, y, z));

                world.add_body(rb);
            }
        }
    }

    /*
     * Set up the testbed.
     */
    let mut testbed = Testbed::new(world);

    testbed.look_at(Pnt3::new(-30.0, 30.0, -30.0), Pnt3::new(0.0, 0.0, 0.0));
    testbed.run();
}
