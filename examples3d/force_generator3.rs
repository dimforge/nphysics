extern crate nalgebra as na;

use na::{Point3, Vector3};
use ncollide3d::shape::{Ball, Plane, ShapeHandle};
use nphysics3d::force_generator::{ConstantAcceleration, DefaultForceGeneratorSet};
use nphysics3d::joint::DefaultJointConstraintSet;
use nphysics3d::object::{
    BodyPartHandle, ColliderDesc, DefaultBodySet, DefaultColliderSet, Ground, RigidBodyDesc,
};
use nphysics3d::world::{DefaultGeometricalWorld, DefaultMechanicalWorld};
use nphysics_testbed3d::{r, Real, Testbed};

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mechanical_world = DefaultMechanicalWorld::new(Vector3::zeros());
    let geometrical_world = DefaultGeometricalWorld::new();
    let mut bodies = DefaultBodySet::new();
    let mut colliders = DefaultColliderSet::new();
    let joint_constraints = DefaultJointConstraintSet::new();
    let mut force_generators = DefaultForceGeneratorSet::new();

    // We setup two force generators that will replace the gravity.
    let mut up_gravity = ConstantAcceleration::new(Vector3::y() * 9.81, Vector3::zeros());
    let mut down_gravity = ConstantAcceleration::new(Vector3::y() * -9.81, Vector3::zeros());

    /*
     * Planes
     */
    // Ground body shared by all floors.
    let ground_handle = bodies.insert(Ground::new());
    let plane = ShapeHandle::new(Plane::new(Vector3::y_axis()));

    let co = ColliderDesc::new(plane).build(BodyPartHandle(ground_handle, 0));
    colliders.insert(co);

    let plane = ShapeHandle::new(Plane::new(-Vector3::y_axis()));
    let co = ColliderDesc::new(plane)
        .translation(Vector3::y() * 20.0)
        .build(BodyPartHandle(ground_handle, 0));
    colliders.insert(co);

    /*
     * Create the balls
     */
    let num = 1000f64.sqrt() as usize;
    let rad = r!(0.1);
    let shift = 0.25 * rad;
    let centerx = shift * r!(num as f32) / r!(2.0);
    let centery = 0.5;

    let ball = ShapeHandle::new(Ball::new(rad));

    for i in 0usize..num {
        for j in 0usize..2 {
            for k in 0usize..num {
                let x = r!(i as f32) * 2.5 * rad - centerx;
                let y = 1.0 + r!(j as f32) * 2.5 * rad + centery;
                let z = r!(k as f32) * 2.5 * rad - centerx;

                // Build the rigid body.
                let rb = RigidBodyDesc::new()
                    .translation(Vector3::new(x, y, z))
                    .build();
                let rb_handle = bodies.insert(rb);

                // Build the collider.
                let co = ColliderDesc::new(ball.clone())
                    .density(r!(1.0))
                    .build(BodyPartHandle(rb_handle, 0));
                colliders.insert(co);

                /*
                 * Set artificial gravity.
                 */
                let color;

                if j == 1 {
                    up_gravity.add_body_part(BodyPartHandle(rb_handle, 0));
                    color = Point3::new(0.0, 0.0, 1.0);
                } else {
                    down_gravity.add_body_part(BodyPartHandle(rb_handle, 0));
                    color = Point3::new(0.0, 1.0, 0.0);
                }

                testbed.set_body_color(rb_handle, color);
            }
        }
    }

    /*
     * Add the force generators to the world.
     */
    force_generators.insert(Box::new(up_gravity));
    force_generators.insert(Box::new(down_gravity));

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
    testbed.look_at(Point3::new(-1.0, 5.0, -1.0), Point3::new(0.0, 0.0, 0.0));
}

fn main() {
    let testbed = Testbed::from_builders(0, vec![("Force generators", init_world)]);
    testbed.run()
}
