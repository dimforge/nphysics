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
extern crate nphysics;
extern crate nphysics_testbed3d;

use na::{Pnt3, Vec3, Translation};
use ncollide::shape::{Plane, Convex};
use ncollide::procedural;
use nphysics::world::World;
use nphysics::object::RigidBody;
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

    world.add_body(RigidBody::new_static(geom, 0.3, 0.6));

    /*
     * Create the convex geometries.
     */
    let num     = 8;
    let shift   = 2.0;
    let centerx = shift * (num as f32) / 2.0;
    let centery = shift / 2.0;
    let centerz = shift * (num as f32) / 2.0;

    for i in 0usize .. num {
        for j in 0usize .. num {
            for k in 0usize .. num {
                let excentricity = 5000.0;
                let x = i as f32 * shift - centerx - excentricity;
                let y = j as f32 * shift + centery - excentricity;
                let z = k as f32 * shift - centerz - excentricity;

                let mut shape = procedural::cuboid(&Vec3::new(2.0 - 0.08, 2.0 - 0.08, 2.0 - 0.08));

                for c in shape.coords.iter_mut() {
                    *c = *c + Vec3::new(excentricity, excentricity, excentricity);
                }

                let geom = Convex::new(shape.coords);
                let mut rb = RigidBody::new_dynamic(geom, 1.0, 0.3, 0.5);
                rb.set_deactivation_threshold(None);

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
