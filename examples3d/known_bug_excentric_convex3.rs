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
extern crate ncollide3d;
extern crate nphysics3d;
extern crate nphysics_testbed3d;

use na::{Point3, Vector3};
use ncollide3d::shape::{ConvexHull, Cuboid, ShapeHandle};

use nphysics3d::world::World;
use nphysics3d::object::{ColliderDesc, RigidBodyDesc};

use nphysics_testbed3d::Testbed;


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
        ShapeHandle::new(Cuboid::new(Vector3::repeat(ground_size)));

    ColliderDesc::new(ground_shape)
        .translation(Vector3::y() * -ground_size)
        .build(&mut world);

    /*
     * Create the convex geometries.
     */
    let num = 8;
    let rad = 0.1;
    let excentricity = 1000.0f32;

    let pts = vec![
        Point3::new(-rad, -rad, -rad) + Vector3::repeat(excentricity),
        Point3::new(-rad, -rad,  rad) + Vector3::repeat(excentricity),
        Point3::new(-rad,  rad, -rad) + Vector3::repeat(excentricity),
        Point3::new(-rad,  rad,  rad) + Vector3::repeat(excentricity),
        Point3::new( rad, -rad, -rad) + Vector3::repeat(excentricity),
        Point3::new( rad, -rad,  rad) + Vector3::repeat(excentricity),
        Point3::new( rad,  rad, -rad) + Vector3::repeat(excentricity),
        Point3::new( rad,  rad,  rad) + Vector3::repeat(excentricity),
    ];

    let shape = ShapeHandle::new(ConvexHull::try_from_points(&pts).unwrap());
    let collider_desc = ColliderDesc::new(shape)
        .density(1.0);

    let mut rb_desc = RigidBodyDesc::new()
        .collider(&collider_desc);

    let shift = (rad + collider_desc.get_margin()) * 2.0;
    let centerx = shift * (num as f32) / 2.0;
    let centery = shift / 2.0;
    let centerz = shift * (num as f32) / 2.0;


    for i in 0usize..num {
        for j in 0usize..num {
            for k in 0usize..num {
                let x = i as f32 * shift - centerx - excentricity;
                let y = j as f32 * shift + centery - excentricity;
                let z = k as f32 * shift - centerz - excentricity;

                // Build the rigid body and its collider.
                rb_desc
                    .set_translation(Vector3::new(x, y, z))
                    .build(&mut world);
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
