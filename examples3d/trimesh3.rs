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

use rand::distributions::{Normal, Distribution};
use rand::thread_rng;



pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut world = World::new();
    world.set_gravity(Vector3::new(0.0, -9.81, 0.0));

    /*
     * Use a fourier series to model ground height. This isn't an ideal terrain
     * model, but is a lot better than using uncorrelated random points.
     */
    let distribution = Normal::new(0.0, 0.5);

    let make_fourier = || {
        let mut rng = thread_rng();
        let a0 = distribution.sample(&mut rng) as f32;
        let a1 = distribution.sample(&mut rng) as f32;
        let b1 = distribution.sample(&mut rng) as f32;
        let a2 = distribution.sample(&mut rng) as f32;
        let b2 = distribution.sample(&mut rng) as f32;
        let a3 = distribution.sample(&mut rng) as f32;
        let b3 = distribution.sample(&mut rng) as f32;
        let tau: f32 = 6.283185307179586 / 50f32;
        move |t: f32| {
            0.5*a0 + a1 * (tau * t).cos() + b1 * (tau * t).sin()
                + a2 * (2.0 * tau * t).cos() + b2 * (2.0 * tau * t).sin()
                + a3 * (3.0 * tau * t).cos() + b3 * (3.0 * tau * t).sin()
        }
    };
    let fourier_x = make_fourier();
    let fourier_y = make_fourier();

    /*
     * Setup a random ground.
     */
    let quad = ncollide3d::procedural::quad(20.0, 20.0, 100, 100);
    let indices = quad
        .flat_indices()
        .chunks(3)
        .map(|is| Point3::new(is[0] as usize, is[2] as usize, is[1] as usize))
        .collect();

    let mut vertices = quad.coords;

    // ncollide generates a quad with `z` as the normal.
    // so we switch z and y here and set a random altitude at each point.
    for p in &mut vertices {
        p.z = p.y;
        p.y = fourier_x(p.x) + fourier_y(p.z);
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
    testbed.set_world(world);
}

fn main() {
    let mut testbed = Testbed::new_empty();
    init_world(&mut testbed);
    testbed.run();
}
