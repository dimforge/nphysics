extern crate nalgebra as na;

use na::{Point2, Vector2, Point3};
use ncollide2d::shape::{Ball, Cuboid, ShapeHandle};
use ncollide2d::query::Proximity;
use nphysics2d::object::{ColliderDesc, RigidBodyDesc, DefaultBodySet, DefaultColliderSet, Ground, BodyPartHandle};
use nphysics2d::force_generator::DefaultForceGeneratorSet;
use nphysics2d::joint::DefaultJointConstraintSet;
use nphysics2d::world::{DefaultMechanicalWorld, DefaultGeometricalWorld};
use nphysics_testbed2d::Testbed;


pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mechanical_world = DefaultMechanicalWorld::new(Vector2::new(0.0, -9.81));
    let geometrical_world = DefaultGeometricalWorld::new();
    let mut bodies = DefaultBodySet::new();
    let mut colliders = DefaultColliderSet::new();
    let joint_constraints = DefaultJointConstraintSet::new();
    let force_generators = DefaultForceGeneratorSet::new();

    /*
     * Ground.
     */
    let ground_size = 10.0;
    let ground_shape =
        ShapeHandle::new(Cuboid::new(Vector2::new(ground_size, 1.0)));

    let ground_handle = bodies.insert(Ground::new());
    let co = ColliderDesc::new(ground_shape)
        .translation(-Vector2::y())
        .build(BodyPartHandle(ground_handle, 0));
    colliders.insert(co);

    /*
     * Create some boxes.
     */
    let num = 15;
    let rad = 0.2;

    let cuboid = ShapeHandle::new(Cuboid::new(Vector2::repeat(rad)));

    let shift = (rad + ColliderDesc::<f32>::default_margin()) * 2.0;
    let centerx = shift * (num as f32) / 2.0;

    for i in 0usize..num {
        let x = i as f32 * shift - centerx;

        // Build the rigid body.
        let rb = RigidBodyDesc::new()
            .translation(Vector2::new(x, 2.0))
            .build();
        let rb_handle = bodies.insert(rb);

        // Build the collider.
        let co = ColliderDesc::new(cuboid.clone())
            .density(1.0)
            .build(BodyPartHandle(rb_handle, 0));
        colliders.insert(co);

        testbed.set_body_color(rb_handle, Point3::new(0.5, 0.5, 1.0));
    }

    /*
     * Create a box that will have a ball-shaped sensor attached.
     */


    let sensor_body = RigidBodyDesc::new()
        .translation(Vector2::new(0.0, 4.0))
        .build();
    let sensor_handle = bodies.insert(sensor_body);

    // Collidable cuboid attached to the sensor body.
    let sensor_collider1 = ColliderDesc::new(cuboid.clone())
        .density(1.0)
        .build(BodyPartHandle(sensor_handle, 0));
    colliders.insert(sensor_collider1);

    // We create a collider desc without density because we don't
    // want it to contribute to the rigid body mass.
    let sensor_geom = ShapeHandle::new(Ball::new(rad * 5.0));
    let sensor_collider2 = ColliderDesc::new(sensor_geom)
        .sensor(true)
        .build(BodyPartHandle(sensor_handle, 0));
    colliders.insert(sensor_collider2);

    testbed.set_body_color(sensor_handle, Point3::new(0.5, 1.0, 1.0));


    // Callback that will be executed on the main loop to handle proximities.
    testbed.add_callback(move |_, geometrical_world, _, colliders, graphics, _| {
        for prox in geometrical_world.proximity_events() {
            let color = match prox.new_status {
                Proximity::WithinMargin | Proximity::Intersecting => Point3::new(1.0, 1.0, 0.0),
                Proximity::Disjoint => Point3::new(0.5, 0.5, 1.0),
            };

            let body1 = colliders.get(prox.collider1).unwrap().body();
            let body2 = colliders.get(prox.collider2).unwrap().body();

            if body1 != ground_handle && body1 != sensor_handle {
                graphics.set_body_color(body1, color);
            }

            if body2 != ground_handle && body2 != sensor_handle {
                graphics.set_body_color(body2, color);
            }
        }
    });


    /*
     * Set up the testbed.
     */
    testbed.set_ground_handle(Some(ground_handle));
    testbed.set_world(mechanical_world, geometrical_world, bodies, colliders, joint_constraints, force_generators);
    testbed.look_at(Point2::origin(), 75.0);
}


fn main() {
    let testbed = Testbed::from_builders(0, vec![
        ("Sensor", init_world),
    ]);
    testbed.run()
}