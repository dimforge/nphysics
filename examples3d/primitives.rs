extern crate nalgebra as na;
extern crate ncollide;
extern crate nphysics3d;
extern crate nphysics_testbed3d;

use na::{Pnt3, Vec3, Translation};
use ncollide::shape::{Plane, Cuboid, Cone, Cylinder, Ball};
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
     * Planes
     */
    let rb = RigidBody::new_static(Plane::new(Vec3::new(0.0, 1.0, 0.0)), 0.3, 0.6);

    world.add_body(rb);

    /*
     * Create the boxes
     */
    let num     = 8;
    let rad     = 1.0;
    let shift   = (rad + 0.08) * 2.0;
    let centerx = shift * (num as f32) / 2.0;
    let centery = shift / 2.0;
    let centerz = shift * (num as f32) / 2.0;

    for i in 0usize .. num {
        for j in 0usize .. num {
            for k in 0usize .. num {
                let x = i as f32 * shift - centerx;
                let y = j as f32 * shift + centery;
                let z = k as f32 * shift - centerz;

                let mut rb;

                if j % 4 == 0 {
                    let geom = Cuboid::new(Vec3::new(rad, rad, rad));
                    rb       = RigidBody::new_dynamic(geom, 1.0, 0.3, 0.5);
                }
                else if j % 3 == 0 {
                    let geom = Ball::new(rad);
                    rb       = RigidBody::new_dynamic(geom, 1.0, 0.3, 0.5);
                }
                else if j % 2 == 0 {
                    let geom = Cylinder::new(rad, rad);
                    rb       = RigidBody::new_dynamic(geom, 1.0, 0.3, 0.5);
                }
                else {
                    let geom = Cone::new(rad, rad);
                    rb       = RigidBody::new_dynamic(geom, 1.0, 0.3, 0.5);
                }

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
