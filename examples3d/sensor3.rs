extern crate nalgebra as na;

use na::{Point3, RealField, Vector3};
use ncollide3d::query::Proximity;
use ncollide3d::shape::{Ball, Cuboid, ShapeHandle};
use nphysics3d::force_generator::DefaultForceGeneratorSet;
use nphysics3d::joint::DefaultJointConstraintSet;
use nphysics3d::object::{
    BodyPartHandle, ColliderDesc, DefaultBodySet, DefaultColliderSet, Ground, RigidBodyDesc,
};
use nphysics3d::world::{DefaultGeometricalWorld, DefaultMechanicalWorld};
use nphysics_testbed3d::Testbed;

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
     * Ground.
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
     * Create some boxes.
     */
    let num = 10;
    let rad = r!(0.2);

    let cuboid = ShapeHandle::new(Cuboid::new(Vector3::repeat(rad)));

    let shift = (rad + ColliderDesc::<N>::default_margin()) * r!(2.0);
    let centerx = shift * r!(num as f64) / r!(2.0);
    let centerz = shift * r!(num as f64) / r!(2.0);

    for i in 0usize..num {
        for k in 0usize..num {
            let x = r!(i as f64) * shift - centerx;
            let z = r!(k as f64) * shift - centerz;

            // Build the rigid body.
            let rb = RigidBodyDesc::new()
                .translation(Vector3::new(x, r!(3.0), z))
                .build();
            let rb_handle = bodies.insert(rb);

            // Build the collider.
            let co = ColliderDesc::new(cuboid.clone())
                .density(r!(1.0))
                .build(BodyPartHandle(rb_handle, 0));
            colliders.insert(co);

            testbed.set_body_color(rb_handle, Point3::new(0.5, 0.5, 1.0));
        }
    }

    /*
     * Create a cube that will have a ball-shaped sensor attached.
     */

    // Rigid body so that the sensor can move.
    let sensor = RigidBodyDesc::new()
        .translation(Vector3::new(r!(0.0), r!(10.0), r!(0.0)))
        .build();
    let sensor_handle = bodies.insert(sensor);

    // Solid cube attached to the sensor which
    // other colliders can touch.
    let co = ColliderDesc::new(cuboid.clone())
        .density(r!(1.0))
        .build(BodyPartHandle(sensor_handle, 0));
    colliders.insert(co);

    // Ball-shaped sensor.
    let sensor_geom = ShapeHandle::new(Ball::new(rad * r!(5.0)));
    // We create a collider desc without density because we don't
    // want it to contribute to the rigid body mass.
    let sensor_collider = ColliderDesc::new(sensor_geom)
        .sensor(true)
        .build(BodyPartHandle(sensor_handle, 0));
    colliders.insert(sensor_collider);

    testbed.set_body_color(sensor_handle, Point3::new(0.5, 1.0, 1.0));

    // Callback that will be executed on the main loop to handle proximities.
    testbed.add_callback(move |_, geometrical_world, _, colliders, graphics, _| {
        for prox in geometrical_world.proximity_events() {
            let color = match prox.new_status {
                Proximity::WithinMargin | Proximity::Intersecting => Point3::new(1.0, 1.0, 0.0),
                Proximity::Disjoint => Point3::new(0.5, 0.5, 1.0),
            };

            let body_handle1 = colliders.get(prox.collider1).unwrap().body();
            let body_handle2 = colliders.get(prox.collider2).unwrap().body();

            if body_handle1 != ground_handle && body_handle1 != sensor_handle {
                graphics.set_body_color(body_handle1, color);
            }
            if body_handle2 != ground_handle && body_handle2 != sensor_handle {
                graphics.set_body_color(body_handle2, color);
            }
        }
    });

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
    testbed.look_at(Point3::new(-6.0, 4.0, -6.0), Point3::new(0.0, 1.0, 0.0));
}

fn main() {
    let testbed = Testbed::<f32>::from_builders(0, vec![("Boxes", init_world)]);

    testbed.run()
}
