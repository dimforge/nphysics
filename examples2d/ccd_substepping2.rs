// Interesting example for testing substepping.

extern crate nalgebra as na;

use na::{Isometry2, Point2, Point3, RealField, Vector2};
use ncollide2d::query::Proximity;
use ncollide2d::shape::{Ball, Compound, Cuboid, ShapeHandle};
use nphysics2d::force_generator::DefaultForceGeneratorSet;
use nphysics2d::joint::DefaultJointConstraintSet;
use nphysics2d::material::{BasicMaterial, MaterialHandle};
use nphysics2d::math::Velocity;
use nphysics2d::object::{
    BodyPartHandle, ColliderDesc, DefaultBodySet, DefaultColliderSet, Ground, RigidBodyDesc,
};
use nphysics2d::world::{DefaultGeometricalWorld, DefaultMechanicalWorld};
use nphysics_testbed2d::Testbed;

/*
 * NOTE: The `r` macro is only here to convert from f64 to the `N` scalar type.
 * This simplifies experimentation with various scalar types (f32, fixed-point numbers, etc.)
 */
pub fn init_world<N: RealField>(testbed: &mut Testbed<N>) {
    /*
     * World
     */
    let mut mechanical_world = DefaultMechanicalWorld::new(Vector2::new(r!(0.0), r!(-9.81)));
    let geometrical_world = DefaultGeometricalWorld::new();
    let mut bodies = DefaultBodySet::new();
    let mut colliders = DefaultColliderSet::new();
    let joint_constraints = DefaultJointConstraintSet::new();
    let force_generators = DefaultForceGeneratorSet::new();

    // Note that setting mechanical_world.integration_parameters.multiple_ccd_substep_sensor_events_enabled
    // to `true` here will have no effect because it will be overwritten by the testbed.

    /*
     * Ground
     */
    let ground_size = 25.0;
    let ground_shape = ShapeHandle::new(Cuboid::new(Vector2::new(ground_size, 0.1)));

    let material = MaterialHandle::new(BasicMaterial::new(2.0, 0.5));

    let ground_handle = bodies.insert(Ground::new());
    let co = ColliderDesc::new(ground_shape.clone())
        .ccd_enabled(true)
        .material(material.clone())
        .build(BodyPartHandle(ground_handle, 0));
    colliders.insert(co);

    let co = ColliderDesc::new(ground_shape.clone())
        .position(Isometry2::new(Vector2::new(-1.0, 0.0), 3.14 / r!(2.0)))
        .ccd_enabled(true)
        .material(material.clone())
        .build(BodyPartHandle(ground_handle, 0));
    colliders.insert(co);

    let co = ColliderDesc::new(ground_shape.clone())
        .position(Isometry2::new(Vector2::new(0.5, 0.0), 3.14 / r!(2.0)))
        .ccd_enabled(true)
        .material(material.clone())
        .build(BodyPartHandle(ground_handle, 0));
    colliders.insert(co);

    let co = ColliderDesc::new(ground_shape.clone())
        .position(Isometry2::translation(0.0, 10.0))
        .ccd_enabled(true)
        .material(material.clone())
        .build(BodyPartHandle(ground_handle, 0));
    colliders.insert(co);

    // Add a sensor, to show that CCD works on sensors too.
    let co = ColliderDesc::new(ground_shape)
        .position(Isometry2::new(Vector2::new(-0.3, 0.0), 3.14 / r!(2.0)))
        .ccd_enabled(true)
        .sensor(true)
        .material(material.clone())
        .build(BodyPartHandle(ground_handle, 0));
    let sensor_handle = colliders.insert(co);
    testbed.set_collider_color(sensor_handle, Point3::new(1.0, 1.0, 0.0));

    /*
     * Create the shapes
     */
    let num = 1; // 5;
    let mut rady = 0.1;
    let mut radx = rady * 4.0;

    let shape = ShapeHandle::new(Ball::new(rady));

    let shiftx = (radx + ColliderDesc::<N>::default_margin() + 0.003) * r!(2.0);
    let shifty = (rady + ColliderDesc::<N>::default_margin() + 0.003) * r!(2.0);
    let centerx = shiftx * r!(num as f64) / r!(2.0) - 0.5;
    let centery = shifty / r!(2.0) + 4.0;

    for i in 0usize..num {
        for j in 0..num {
            let x = r!(i as f64) * shiftx - centerx;
            let y = r!(j as f64) * shifty + centery;

            // Build the rigid body.
            let rb = RigidBodyDesc::new()
                .translation(Vector2::new(x, y))
                .velocity(Velocity::linear(1000.0, -1000.0))
                .build();
            let rb_handle = bodies.insert(rb);

            // Build the collider.
            let co = ColliderDesc::new(shape.clone())
                .ccd_enabled(true)
                .density(r!(1.0))
                .build(BodyPartHandle(rb_handle, 0));
            colliders.insert(co);

            testbed.set_body_color(rb_handle, Point3::new(0.5, 0.5, 1.0));
        }
    }

    /*
     * Set up the testbed.
     */

    // Callback that will be executed on the main loop to handle proximities.
    testbed.add_callback(move |_, geometrical_world, _, colliders, graphics, _| {
        for prox in geometrical_world.proximity_events() {
            println!(
                "Detected proximity {:?} between {:?} and {:?}.",
                prox.new_status, prox.collider1, prox.collider2
            );
            let c1 = colliders.get(prox.collider1).unwrap();
            let c2 = colliders.get(prox.collider2).unwrap();
            let body1 = c1.body();
            let body2 = c2.body();

            let color = match prox.new_status {
                Proximity::WithinMargin | Proximity::Intersecting => Point3::new(1.0, 1.0, 0.0),
                Proximity::Disjoint => {
                    if c1.position().translation.vector.x > c2.position().translation.vector.x {
                        if body1 == ground_handle {
                            Point3::new(0.5, 0.5, 1.0)
                        } else {
                            Point3::new(0.5, 1.0, 0.5)
                        }
                    } else {
                        if body1 == ground_handle {
                            Point3::new(0.5, 1.0, 0.5)
                        } else {
                            Point3::new(0.5, 0.5, 1.0)
                        }
                    }
                }
            };

            if body1 != ground_handle {
                graphics.set_body_color(body1, color);
            }

            if body2 != ground_handle {
                graphics.set_body_color(body2, color);
            }
        }
    });

    testbed.set_ground_handle(Some(ground_handle));
    testbed.set_world(
        mechanical_world,
        geometrical_world,
        bodies,
        colliders,
        joint_constraints,
        force_generators,
    );
    testbed.look_at(Point2::new(3.0, 5.0), 95.0);
}

fn main() {
    let testbed = Testbed::<f32>::from_builders(0, vec![("CCD substepping", init_world)]);
    testbed.run()
}
