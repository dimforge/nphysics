extern crate nalgebra as na;
extern crate ncollide3d;
extern crate nphysics3d;
extern crate nphysics_testbed3d;

use na::{Isometry3, Point3, Unit, Vector3};
use ncollide3d::shape::{Ball, Plane, ShapeHandle};
use nphysics3d::volumetric::Volumetric;
use nphysics3d::world::World;
use nphysics3d::object::{BodyPartHandle, Material};
use nphysics_testbed3d::Testbed;

const COLLIDER_MARGIN: f32 = 0.01;

fn main() {
    /*
     * World
     */
    let mut world = World::new();
    world.set_gravity(Vector3::new(0.0, -9.81, 0.0));

    // Material.
    let material = Material::default();

    /*
     * Planes
     */
    let normals = [
        Unit::new_normalize(Vector3::new(-1.0, 1.0, -1.0)),
        Unit::new_normalize(Vector3::new(1.0, 1.0, -1.0)),
        Unit::new_normalize(Vector3::new(-1.0, 1.0, 1.0)),
        Unit::new_normalize(Vector3::new(1.0, 1.0, 1.0)),
    ];
    for n in normals.iter() {
        let ground_shape = ShapeHandle::new(Plane::new(*n));
        let ground_pos = Isometry3::identity();

        world.add_collider(
            COLLIDER_MARGIN,
            ground_shape,
            BodyPartHandle::ground(),
            ground_pos,
            material.clone(),
        );
    }

    /*
     * Create the balls
     */
    let num = 1500.0f32.powf(1.0f32 / 3.0) as usize;
    let rad = 0.1;
    let shift = 2.5 * rad;
    let centerx = shift * (num as f32) / 2.0;
    let centery = shift * (num as f32) / 2.0;

    let geom = ShapeHandle::new(Ball::new(rad - COLLIDER_MARGIN));
    let inertia = geom.inertia(1.0);
    let center_of_mass = geom.center_of_mass();

    for i in 0usize..num {
        for j in 0usize..num {
            for k in 0usize..num {
                let x = i as f32 * 2.5 * rad - centerx;
                let y = 3.0 + j as f32 * 2.5 * rad + centery * 2.0;
                let z = k as f32 * 2.5 * rad - centerx;

                /*
                 * Create the rigid body.
                 */
                let pos = Isometry3::new(Vector3::new(x, y, z), na::zero());
                let handle = world.add_rigid_body(pos, inertia, center_of_mass);

                /*
                 * Create the collider.
                 */
                world.add_collider(
                    COLLIDER_MARGIN,
                    geom.clone(),
                    handle,
                    Isometry3::identity(),
                    material.clone(),
                );
            }
        }
    }

    /*
     * Set up the testbed.
     */
    let mut testbed = Testbed::new(world);

    testbed.look_at(Point3::new(-1.0, 15.0, -1.0), Point3::new(0.0, 0.0, 0.0));
    testbed.run();
}
