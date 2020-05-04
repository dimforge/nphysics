extern crate nalgebra as na;

use na::{Point3, RealField, Vector3};
use ncollide3d::shape::{Cuboid, ShapeHandle, TriMesh};
use nphysics3d::force_generator::DefaultForceGeneratorSet;
use nphysics3d::joint::DefaultJointConstraintSet;
use nphysics3d::object::{
    BodyPartHandle, ColliderDesc, DefaultBodySet, DefaultColliderSet, Ground, RigidBodyDesc,
};
use nphysics3d::world::{DefaultGeometricalWorld, DefaultMechanicalWorld};
use nphysics_testbed3d::Testbed;

use rand::thread_rng;
use rand_distr::{Distribution, Normal};

pub fn init_world<N: RealField>(testbed: &mut Testbed<N>) {
    /*
     * World
     */
    let mechanical_world = DefaultMechanicalWorld::new(Vector3::new(r!(0.0), r!(-9.81), r!(0.0)));
    let geometrical_world = DefaultGeometricalWorld::new();
    let mut bodies = DefaultBodySet::new();
    let mut colliders = DefaultColliderSet::new();
    let joint_constraints = DefaultJointConstraintSet::new();
    let force_generators = DefaultForceGeneratorSet::new();

    /*
     * Use a fourier series to model ground height. This isn't an ideal terrain
     * model, but is a lot better than using uncorrelated random points.
     */
    let distribution = Normal::new(0.0, 0.5).unwrap();

    let make_fourier = || {
        let mut rng = thread_rng();
        let a0 = r!(distribution.sample(&mut rng));
        let a1 = r!(distribution.sample(&mut rng));
        let b1 = r!(distribution.sample(&mut rng));
        let a2 = r!(distribution.sample(&mut rng));
        let b2 = r!(distribution.sample(&mut rng));
        let a3 = r!(distribution.sample(&mut rng));
        let b3 = r!(distribution.sample(&mut rng));
        let tau: N = r!(6.283185307179586 / 50.0);
        move |t: N| {
            r!(0.5) * a0
                + a1 * (tau * t).cos()
                + b1 * (tau * t).sin()
                + a2 * (r!(2.0) * tau * t).cos()
                + b2 * (r!(2.0) * tau * t).sin()
                + a3 * (r!(3.0) * tau * t).cos()
                + b3 * (r!(3.0) * tau * t).sin()
        }
    };
    let fourier_x = make_fourier();
    let fourier_y = make_fourier();

    /*
     * Setup a random ground.
     */
    let quad = ncollide3d::procedural::quad(r!(20.0), r!(20.0), 100, 100);
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

    let trimesh: TriMesh<N> = TriMesh::new(vertices, indices, None);
    let ground_handle = bodies.insert(Ground::new());
    let co = ColliderDesc::new(ShapeHandle::new(trimesh)).build(BodyPartHandle(ground_handle, 0));
    colliders.insert(co);

    /*
     * Create some boxes and spheres.
     */
    let num = 7;
    let rad = r!(0.1);
    let shift = rad * r!(2.0) + r!(0.5);
    let centerx = shift * r!(num as f64) / r!(2.0);
    let centery = shift / r!(2.0);
    let centerz = shift * r!(num as f64) / r!(2.0);
    let height = r!(2.0);

    let cuboid = ShapeHandle::new(Cuboid::new(Vector3::repeat(rad)));

    for i in 0usize..num {
        for j in 0usize..num {
            for k in 0usize..num {
                let x = r!(i as f64) * shift - centerx;
                let y = r!(j as f64) * shift + centery + height;
                let z = r!(k as f64) * shift - centerz;

                // Build the rigid body.
                let rb = RigidBodyDesc::new()
                    .translation(Vector3::new(x, y, z))
                    .build();
                let rb_handle = bodies.insert(rb);

                // Build the collider.
                let co = ColliderDesc::new(cuboid.clone())
                    .density(r!(1.0))
                    .build(BodyPartHandle(rb_handle, 0));
                colliders.insert(co);
            }
        }
    }

    /*
     * Set up the testbed.
     */
    testbed.set_ground_handle(Some(ground_handle));
    testbed.set_world(
        mechanical_world,
        geometrical_world,
        bodies,
        colliders,
        joint_constraints,
        force_generators,
    );
    testbed.look_at(Point3::new(-20.0, 20.0, -20.0), Point3::new(0.0, 1.0, 0.0));
}

fn main() {
    let testbed = Testbed::<f32>::from_builders(0, vec![("Triangle mesh", init_world)]);

    testbed.run()
}
