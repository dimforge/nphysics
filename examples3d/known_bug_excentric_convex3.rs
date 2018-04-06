/*!
 * # Expected behaviour:
 * Same as the box_vee3d demo.
 *
 * It seems to behave as expected if the excentricity is not too big (tested with 10 and 100).
 *
 * # Symptoms:
 * Some object just fall through the ground, missing any collison. Then, after a while they seem
 * to notice that they are deep into the plane, and "jump" due to penetration resolution.
 * Thus, some collision are missed.
 * Identically, some boxes just dont collide to each other some times.
 *
 * # Cause:
 * Not sure, but this seems to be an accuracy limitation of the contact manifold generators
 * (OneShotContactManifoldGenerator and IncrementalContactManifoldGenerator). The repetitive
 * transformations of the saved contact might invalidate them.
 *
 * However, this might be a real bug for some other non-accuracy-related reasons. For example, when
 * a box is deep on the plane without contact why does the one-shot contact manifold generator
 * fails? Something wrong with the perturbation?
 *
 * This might be (but it is very unlikely) a problem with the DBVT that might become invalid.
 * Though this seems very unlikely as the AABBs seem to be fine and the plane has an infinite aabb
 * anyway. Thurthermore, the ray-cast (which uses the dbvt…) works fine, even for "jumpy" objects.
 *
 *
 * # Solution:
 *
 *
 * # Limitations of the solution:
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
    let shift = 2.0;
    let excentricity = 5000.0f32;
    let centerx = shift * (num as f32) / 2.0;
    let centery = shift / 2.0;
    let centerz = shift * (num as f32) / 2.0;

    let mut cuboid_mesh = procedural::cuboid(&Vector3::new(2.0 - 0.08, 2.0 - 0.08, 2.0 - 0.08));

    for c in cuboid_mesh.coords.iter_mut() {
        *c = *c + Vector3::new(excentricity, excentricity, excentricity);
    }

    let indices: Vec<usize> = cuboid_mesh.flat_indices()
        .into_iter()
        .map(|i| i as usize)
        .collect();
    let vertices = cuboid_mesh.coords;

    for v in &vertices {
        println!("Vertex: {}", *v)
    }

    for i in indices.chunks(3) {
        println!("Index: {}, {}, {}", i[0], i[1], i[2])
    }
    println!("Original indices: {:?}", cuboid_mesh.indices);

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

    testbed.look_at(Point3::new(-30.0, 30.0, -30.0), Point3::new(0.0, 0.0, 0.0));
    testbed.run();
}
