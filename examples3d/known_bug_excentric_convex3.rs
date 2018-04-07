/*!
 * # Expected behaviour:
 * Same as the box_vee3d demo.
 *
 * It seems to behave as expected if the excentricity is not too big (tested with 10 and 100).
 *
 * # Symptoms:
 * Some objets jitter when they touch the ground.
 *
 * # Cause:
 * Not sure, but this seems to be an accuracy limitation of the contact manifold generators.
 *
 * This might be (but it is very unlikely) a problem with the DBVT that might become invalid.
 * Though this seems very unlikely as the AABBs seem to be fine and the plane has an infinite aabb
 * anyway. Thurthermore, the ray-cast (which uses the dbvt…) works fine, even for "jumpy" objects.
 *
 *
 * # Solution:
 *
 */

extern crate nalgebra as na;
extern crate ncollide;
extern crate nphysics3d;
extern crate nphysics_testbed3d;

use na::{Isometry3, Point3, Vector3};
use ncollide::shape::{ConvexHull, Cuboid, Plane, ShapeHandle};
use ncollide::procedural;
use nphysics3d::world::World;
use nphysics3d::object::{BodyHandle, Material};
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
     * Ground
     */
    let ground_size = 50.0;
    let ground_shape =
        ShapeHandle::new(Cuboid::new(Vector3::repeat(ground_size - COLLIDER_MARGIN)));
    let ground_pos = Isometry3::new(Vector3::y() * -ground_size, na::zero());

    world.add_collider(
        COLLIDER_MARGIN,
        ground_shape,
        BodyHandle::ground(),
        ground_pos,
        Material::default(),
    );

    /*
     * Create the convex geometries.
     */
    let num = 8;
    let rad = 0.1;
    let shift = rad * 2.0;
    let excentricity = 1000.0f32;
    let centerx = shift * (num as f32) / 2.0;
    let centery = shift / 2.0;
    let centerz = shift * (num as f32) / 2.0;

    let mut cuboid_mesh = procedural::cuboid(&Vector3::repeat(2.0 * (rad - COLLIDER_MARGIN)));

    for c in cuboid_mesh.coords.iter_mut() {
        *c = *c + Vector3::new(excentricity, excentricity, excentricity);
    }

    let indices: Vec<usize> = cuboid_mesh.flat_indices()
        .into_iter()
        .map(|i| i as usize)
        .collect();
    let vertices = cuboid_mesh.coords;

    let geom = ShapeHandle::new(ConvexHull::try_new(vertices, &indices).unwrap());
    let inertia = geom.inertia(1.0);
    let center_of_mass = geom.center_of_mass();

    for i in 0usize..num {
        for j in 0usize..num {
            for k in 0usize..num {
                let x = i as f32 * shift - centerx - excentricity;
                let y = j as f32 * shift + centery - excentricity;
                let z = k as f32 * shift - centerz - excentricity;

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

    testbed.look_at(Point3::new(-10.0, 10.0, -10.0), Point3::new(0.0, 0.0, 0.0));
    testbed.run();
}
