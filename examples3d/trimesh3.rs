extern crate nalgebra as na;
extern crate ncollide3d;
extern crate nphysics3d;
extern crate nphysics_testbed3d;
extern crate rand;

use na::{Point3, Vector3};
use ncollide3d::shape::{Cuboid, ShapeHandle, TriMesh};
use nphysics3d::object::{ColliderDesc, RigidBodyDesc};
use nphysics3d::world::World;
use nphysics_testbed3d::Testbed;

use rand::distributions::{Standard, Distribution};
use rand::{SeedableRng, XorShiftRng};



fn main() {
    /*
     * World
     */
    let mut world = World::new();
    world.set_gravity(Vector3::new(0.0, -9.81, 0.0));

    /*
     * Setup a random ground.
     */
    let quad = ncollide3d::procedural::quad(10.0, 10.0, 10, 10);
    let indices = quad
        .flat_indices()
        .chunks(3)
        .map(|is| Point3::new(is[0] as usize, is[2] as usize, is[1] as usize))
        .collect();

    let mut rng = XorShiftRng::seed_from_u64(42);
    let distribution = Standard;

    let mut vertices = quad.coords;

    // ncollide generatse a quad with `z` as the normal.
    // so we switch z and y here and set a random altitude at each point.
    for p in &mut vertices {
        p.z = p.y;
        let y: f32 = distribution.sample(&mut rng);
        p.y = y * 1.5;
    }

    let trimesh: TriMesh<f32> = TriMesh::new(vertices, indices, None);
    ColliderDesc::new(ShapeHandle::new(trimesh))
        .build(&mut world);

    /*
     * Create some boxes and spheres.
     */
    let num = 7;
    let rad = 0.1;
    let shift = rad * 2.0 + 0.5;
    let centerx = shift * (num as f32) / 2.0;
    let centery = shift / 2.0;
    let centerz = shift * (num as f32) / 2.0;
    let height = 1.0;

    let cuboid = ShapeHandle::new(Cuboid::new(Vector3::repeat(rad)));
    let collider_desc = ColliderDesc::new(cuboid)
        .density(1.0);

    let mut rb_desc = RigidBodyDesc::new()
        .collider(&collider_desc);

    for i in 0usize..num {
        for j in 0usize..num {
            for k in 0usize..num {
                let x = i as f32 * shift - centerx;
                let y = j as f32 * shift + centery + height;
                let z = k as f32 * shift - centerz;

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
    let testbed = Testbed::new(world);
    testbed.run();
}
