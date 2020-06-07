extern crate nalgebra as na;

use na::{Point3, RealField, Vector3};
use ncollide3d::shape::{ConvexHull, Cuboid, ShapeHandle};
use nphysics3d::force_generator::DefaultForceGeneratorSet;
use nphysics3d::joint::DefaultJointConstraintSet;
use nphysics3d::object::{
    BodyPartHandle, ColliderDesc, DefaultBodySet, DefaultColliderSet, Ground, RigidBodyDesc,
};
use nphysics3d::world::{DefaultGeometricalWorld, DefaultMechanicalWorld};
use nphysics_testbed3d::Testbed;

use rand::distributions::{Distribution, Standard};
use rand::{rngs::StdRng, SeedableRng};

/*
 * NOTE: The `r` macro is only here to convert from f64 to the `N` scalar type.
 * This simplifies experimentation with various scalar types (f32, fixed-point numbers, etc.)
 */
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
     * Ground
     */
    let ground_thickness = r!(0.2);
    let ground_shape = ShapeHandle::new(Cuboid::new(Vector3::new(
        r!(3.0),
        ground_thickness,
        r!(3.0),
    )));

    let ground_handle = bodies.insert(Ground::new());
    let co = ColliderDesc::new(ground_shape)
        .translation(Vector3::y() * -ground_thickness)
        .build(BodyPartHandle(ground_handle, 0));
    colliders.insert(co);

    /*
     * Create the convex geometries.
     */
    let npts = 10usize;
    let num = 6;
    let shift = r!(0.4);
    let centerx = shift * r!(num as f64) / r!(2.0);
    let centery = shift / r!(2.0);
    let centerz = shift * r!(num as f64) / r!(2.0);
    let mut rng = StdRng::seed_from_u64(0);
    let distribution = Standard;

    for i in 0usize..num {
        for j in 0usize..num {
            for k in 0usize..num {
                let x = r!(i as f64) * shift - centerx;
                let y = r!(j as f64) * shift + centery;
                let z = r!(k as f64) * shift - centerz;

                let mut pts = Vec::with_capacity(npts);

                for _ in 0..npts {
                    let pt: Point3<f64> = distribution.sample(&mut rng);
                    pts.push((na::convert::<_, Point3<N>>(pt) * r!(0.4)).into());
                }

                // Build the rigid body.
                let rb = RigidBodyDesc::new()
                    .translation(Vector3::new(x, y, z))
                    .build();
                let rb_handle = bodies.insert(rb);

                // Build the collider.
                if let Some(chull) = ConvexHull::try_from_points(&pts) {
                    let geom = ShapeHandle::new(chull);
                    let co = ColliderDesc::new(geom)
                        .density(r!(1.0))
                        .build(BodyPartHandle(rb_handle, 0));
                    colliders.insert(co);
                }
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
    testbed.look_at(Point3::new(-4.0, 1.0, -4.0), Point3::new(0.0, 1.0, 0.0));
}

fn main() {
    let testbed = Testbed::<f32>::from_builders(0, vec![("Convex", init_world)]);

    testbed.run()
}
