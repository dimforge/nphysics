extern crate nalgebra as na;

use na::{Point3, Vector2};
use ncollide2d::shape::{Ball, Cuboid, ShapeHandle};
use nphysics2d::force_generator::{ConstantAcceleration, DefaultForceGeneratorSet};
use nphysics2d::joint::DefaultJointConstraintSet;
use nphysics2d::object::{
    BodyPartHandle, ColliderDesc, DefaultBodySet, DefaultColliderSet, Ground, RigidBodyDesc,
};
use nphysics2d::world::{DefaultGeometricalWorld, DefaultMechanicalWorld};
use nphysics_testbed2d::Testbed;

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mechanical_world = DefaultMechanicalWorld::new(Vector2::zeros());
    let geometrical_world = DefaultGeometricalWorld::new();
    let mut bodies = DefaultBodySet::new();
    let mut colliders = DefaultColliderSet::new();
    let joint_constraints = DefaultJointConstraintSet::new();
    let mut force_generators = DefaultForceGeneratorSet::new();

    // We setup two force generators that will replace the gravity.
    let mut up_gravity = ConstantAcceleration::new(Vector2::y() * -9.81, 0.0);
    let mut down_gravity = ConstantAcceleration::new(Vector2::y() * 9.81, 0.0);

    /*
     * Grouds
     */
    let ground_radx = 25.0;
    let ground_rady = 1.0;
    let ground_shape = ShapeHandle::new(Cuboid::new(Vector2::new(ground_radx, ground_rady)));

    // Ground body shared by the two floors.
    let ground_handle = bodies.insert(Ground::new());

    let mut ground_desc = ColliderDesc::new(ground_shape);

    let ground_collider = ground_desc
        .set_translation(-Vector2::y() * 2.0)
        .build(BodyPartHandle(ground_handle, 0));
    colliders.insert(ground_collider);

    let ground_collider = ground_desc
        .set_translation(Vector2::y() * 3.0)
        .build(BodyPartHandle(ground_handle, 0));
    colliders.insert(ground_collider);

    /*
     * Create the balls
     */
    let num = 100f64 as usize;
    let rad = 0.2;
    let shift = 2.0 * rad;
    let centerx = shift * (num as f32) / 2.0;
    let centery = rad * 4.0;

    let ball = ShapeHandle::new(Ball::new(rad));

    for i in 0usize..num {
        for j in 0usize..2 {
            let x = i as f32 * 2.5 * rad - centerx;
            let y = j as f32 * 2.5 * -rad + centery;

            // Build the rigid body.
            let rb = RigidBodyDesc::new().translation(Vector2::new(x, y)).build();
            let rb_handle = bodies.insert(rb);

            // Build the collider.
            let co = ColliderDesc::new(ball.clone())
                .density(1.0)
                .build(BodyPartHandle(rb_handle, 0));
            colliders.insert(co);

            /*
             * Set artifical gravity.
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
}

fn main() {
    let testbed = Testbed::from_builders(0, vec![("Force generators", init_world)]);
    testbed.run()
}
