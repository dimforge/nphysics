extern crate nalgebra as na;

use na::{Point3, Vector3};
use ncollide3d::shape::{Cuboid, TriMesh, ShapeHandle};
use nphysics3d::object::{ColliderDesc, RigidBodyDesc, DefaultBodySet, DefaultColliderSet, Ground, BodyPartHandle};
use nphysics3d::force_generator::DefaultForceGeneratorSet;
use nphysics3d::joint::DefaultJointConstraintSet;
use nphysics3d::world::{DefaultDynamicWorld, DefaultColliderWorld};
use nphysics_testbed3d::Testbed;

use rand::distributions::{Normal, Distribution};
use rand::thread_rng;



pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let dynamic_world = DefaultDynamicWorld::new(Vector3::new(0.0, -9.81, 0.0));
    let collider_world = DefaultColliderWorld::new();
    let mut bodies = DefaultBodySet::new();
    let mut colliders = DefaultColliderSet::new();
    let joint_constraints = DefaultJointConstraintSet::new();
    let force_generators = DefaultForceGeneratorSet::new();


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
    let ground_handle = bodies.insert(Ground::new());
    let co = ColliderDesc::new(ShapeHandle::new(trimesh))
        .build(BodyPartHandle(ground_handle, 0));
    colliders.insert(co);

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

    for i in 0usize..num {
        for j in 0usize..num {
            for k in 0usize..num {
                let x = i as f32 * shift - centerx;
                let y = j as f32 * shift + centery + height;
                let z = k as f32 * shift - centerz;

                // Build the rigid body.
                let rb = RigidBodyDesc::new()
                    .translation(Vector3::new(x, y, z))
                    .build();
                let rb_handle = bodies.insert(rb);

                // Build the collider.
                let co = ColliderDesc::new(cuboid.clone())
                    .density(1.0)
                    .build(BodyPartHandle(rb_handle, 0));
                colliders.insert(co);
            }
        }
    }

    /*
     * Set up the testbed.
     */
    testbed.set_ground_handle(Some(ground_handle));
    testbed.set_world(dynamic_world, collider_world, bodies, colliders, joint_constraints, force_generators);
    testbed.look_at(Point3::new(-4.0, 1.0, -4.0), Point3::new(0.0, 1.0, 0.0));
}

fn main() {
    let testbed = Testbed::from_builders(0, vec![
        ("Triangle mesh", init_world),
    ]);

    testbed.run()
}
