extern crate nalgebra as na;

use na::{Point2, RealField, Vector2};
use ncollide2d::shape::{Cuboid, ShapeHandle};
use nphysics2d::force_generator::DefaultForceGeneratorSet;
use nphysics2d::joint::DefaultJointConstraintSet;
use nphysics2d::material::{BasicMaterial, MaterialHandle};
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
    let mechanical_world = DefaultMechanicalWorld::new(Vector2::new(r!(0.0), r!(-9.81)));
    let geometrical_world = DefaultGeometricalWorld::new();
    let mut bodies = DefaultBodySet::new();
    let mut colliders = DefaultColliderSet::new();
    let joint_constraints = DefaultJointConstraintSet::new();
    let force_generators = DefaultForceGeneratorSet::new();

    /*
     * Setup a static body used as the ground.
     */
    let ground_handle = bodies.insert(Ground::new());

    /*
     * Ground
     */
    let ground_size = r!(5.0);
    let ground_shape = ShapeHandle::new(Cuboid::new(Vector2::new(ground_size, r!(0.2))));
    let conveyor_material1 = BasicMaterial {
        surface_velocity: Some(Vector2::x()),
        ..BasicMaterial::default()
    };
    let conveyor_material2 = BasicMaterial {
        surface_velocity: Some(-Vector2::x()),
        ..BasicMaterial::default()
    };

    for i in 0..10 {
        let co = ColliderDesc::new(ground_shape.clone())
            .translation(Vector2::new(r!(-2.0), r!(5.0) - r!(i as f64) * r!(4.0)))
            .rotation(r!(0.1))
            .material(MaterialHandle::new(conveyor_material1))
            .build(BodyPartHandle(ground_handle, 0));
        colliders.insert(co);

        let co = ColliderDesc::new(ground_shape.clone())
            .translation(Vector2::new(r!(2.0), r!(3.0) - r!(i as f64) * r!(4.0)))
            .rotation(r!(-0.1))
            .material(MaterialHandle::new(conveyor_material2))
            .build(BodyPartHandle(ground_handle, 0));
        colliders.insert(co);
    }

    /*
     * Create the boxes
     */
    let num = 5;
    let rad = r!(0.1);

    let cuboid = ShapeHandle::new(Cuboid::new(Vector2::repeat(rad)));

    let shift = (rad + ColliderDesc::<N>::default_margin()) * r!(2.0);
    let centerx = shift * r!(num as f64) / r!(2.0) + r!(5.0);
    let centery = shift / r!(2.0) + r!(5.0);

    for i in 0usize..num {
        for j in 0..num {
            let x = r!(i as f64) * shift - centerx;
            let y = r!(j as f64) * shift + centery;

            // Build the rigid body.
            let rb = RigidBodyDesc::new().translation(Vector2::new(x, y)).build();
            let rb_handle = bodies.insert(rb);

            // Build the collider.
            let co = ColliderDesc::new(cuboid.clone())
                .density(r!(1.0))
                .build(BodyPartHandle(rb_handle, 0));
            colliders.insert(co);
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
    testbed.look_at(Point2::new(0.0, 2.5), 95.0);
}

fn main() {
    let testbed = Testbed::<f32>::from_builders(0, vec![("Conveyor belt", init_world)]);
    testbed.run()
}
