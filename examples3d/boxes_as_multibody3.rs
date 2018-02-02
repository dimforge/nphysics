extern crate nalgebra as na;
extern crate ncollide;
extern crate nphysics3d;
extern crate nphysics_testbed3d;

use std::f32::consts::PI;
use na::{Point3, Vector3, Translation3, Isometry3};
use ncollide::shape::{ShapeHandle, Plane, Cuboid, Ball};
use nphysics3d::world::World;
use nphysics3d::object::{RigidBody, BodyHandle};
use nphysics3d::joint::FreeJoint;
use nphysics3d::math::Twist;
use nphysics3d::volumetric::Volumetric;
use nphysics_testbed3d::Testbed;

const COLLIDER_MARGIN: f32 = 0.01;

fn main() {
    /*
     * World
     */
    let mut world = World::new();
    world.set_gravity(Vector3::new(0.0, -9.81, 0.0));

    /*
     * Plane
     */
    let plane = ShapeHandle::new(Plane::new(Vector3::new(0.0, 1.0, 0.0)));
    world.add_collider(COLLIDER_MARGIN, plane, BodyHandle::ground(), Isometry3::identity(), false);

    /*
     * Create the boxes
     */
    let num     = 8;
    let rad     = 0.2;
    let shift   = rad * 2.0 /*+ 1.0e-4*/;
    let centerx = shift * (num as f32) / 2.0;
    let centery = shift / 2.0 + 0.04;
    let centerz = shift * (num as f32) / 2.0;

    let geom    = ShapeHandle::new(Cuboid::new(Vector3::repeat(rad - COLLIDER_MARGIN)));
    // let geom    = ShapeHandle::new(Ball::new(rad - COLLIDER_MARGIN));
    let inertia = geom.inertia(1.0);

    for i in 0usize .. num {
        for j in 0usize .. num {
            for k in 0usize .. num {
                let x = i as f32 * shift - centerx;
                let y = j as f32 * shift + centery;
                let z = k as f32 * shift - centerz;

                /*
                 * Create the rigid body.
                 */
                let free   = FreeJoint::new(Isometry3::new(Vector3::new(x, y, z), na::zero()));
                let handle = world.add_multibody_link(BodyHandle::ground(), free, Isometry3::identity(), inertia);

                /*
                 * Create the collider.
                 */
                world.add_collider(COLLIDER_MARGIN, geom.clone(), handle, Isometry3::identity(), true);
            }
        }
    }

    /*
     * Set up the testbed.
     */
    let mut testbed = Testbed::new(world);

    testbed.look_at(Point3::new(-10.0, 10.0, -10.0), Point3::new(0.0, 0.0, 0.0));
    testbed.run();
}
