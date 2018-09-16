extern crate nalgebra as na;
extern crate ncollide3d;
extern crate nphysics3d;
extern crate nphysics_testbed3d;

use std::sync::Arc;
use na::{Isometry3, Point3, Vector3};
use ncollide3d::shape::{Cuboid, ShapeHandle};
use nphysics3d::object::{Material, DeformableVolume, BodyStatus};
use nphysics3d::volumetric::Volumetric;
use nphysics3d::world::World;
use nphysics_testbed3d::Testbed;

const COLLIDER_MARGIN: f32 = 0.01;

fn main() {
    /*
     * World
     */
    let mut world = World::new();
    world.set_gravity(Vector3::new(0.0, -9.81, 0.0));

    /*
     * For the ground, create a kinematic deformable body and a collider for its contour.
     */
    let volume = DeformableVolume::cube(
        &Isometry3::identity(),
        &Vector3::new(5.0, 0.2, 5.0),
        6, 2, 2,
        1.0, 1.0e2, 0.3,
        (0.4, 0.0));
    let (mesh, ids_map, parts_map) = volume.boundary_mesh();

    let handle = world.add_body(Box::new(volume));

    // Make it kinematic.
    world.body_mut(handle).set_status(BodyStatus::Static);
    world.add_deformable_collider(
        COLLIDER_MARGIN,
        mesh,
        handle,
        Arc::new(ids_map),
        Arc::new(parts_map),
        Material::default(),
    );

    /*
     * Create boxes
     */
    let num = 5;
    let rad = 0.1;
    let shift = rad * 2.0;
    let centerx = shift * (num as f32) / 2.0;
    let centery = shift / 2.0;
    let centerz = shift * (num as f32) / 2.0;
    let height = 2.0; // 3.0;

    let geom = ShapeHandle::new(Cuboid::new(Vector3::repeat(rad - COLLIDER_MARGIN)));
    let inertia = geom.inertia(1.0);
    let center_of_mass = geom.center_of_mass();

    for i in 0usize..num {
        for j in 0usize..num {
            for k in 0usize..num {
                let x = i as f32 * shift - centerx;
                let y = j as f32 * shift + centery + height;
                let z = k as f32 * shift - centerz;

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
                    Material::default(),
                );
            }
        }
    }


    /*
     * Set up the testbed.
     */
    let mut testbed = Testbed::new(world);
    // testbed.hide_performance_counters();
    testbed.look_at(Point3::new(0.0, 0.0, 2.0), Point3::new(0.0, 0.0, 0.0));
    testbed.run();
}
