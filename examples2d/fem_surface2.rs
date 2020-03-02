extern crate nalgebra as na;

use na::{Point2, Vector2};
use ncollide2d::shape::{Cuboid, ShapeHandle};
use nphysics2d::force_generator::DefaultForceGeneratorSet;
use nphysics2d::joint::DefaultJointConstraintSet;
use nphysics2d::object::{
    BodyPartHandle, ColliderDesc, DefaultBodySet, DefaultColliderSet, FEMSurfaceDesc, Ground,
    RigidBodyDesc,
};
use nphysics2d::world::{DefaultGeometricalWorld, DefaultMechanicalWorld};
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
    // Ground body shared to which both obstacle colliders will be attached.
    let ground_handle = bodies.insert(Ground::new());

    let obstacle = ShapeHandle::new(Cuboid::new(Vector2::repeat(0.2)));
    let mut obstacle_desc = ColliderDesc::new(obstacle);

    let co = obstacle_desc
        .set_translation(Vector2::x() * 4.0)
        .build(BodyPartHandle(ground_handle, 0));
    colliders.insert(co);

    let co = obstacle_desc
        .set_translation(Vector2::x() * -4.0)
        .build(BodyPartHandle(ground_handle, 0));
    colliders.insert(co);

    /*
     * Create the deformable body and a collider for its boundary.
     */
    let mut deformable = FEMSurfaceDesc::quad(50, 1)
        .scale(Vector2::new(10.0, 1.0))
        .translation(Vector2::y() * 1.0)
        .young_modulus(1.0e4)
        .mass_damping(0.2)
        .build();
    let collider_desc = deformable.boundary_collider_desc();
    let deformable_handle = bodies.insert(deformable);

    let co = collider_desc.build(deformable_handle);
    colliders.insert(co);

    /*
     * Create a pyramid on top of the deformable body.
     */
    let num = 10;
    let rad = 0.1;
    let shift = 2.0 * rad;
    let centerx = shift * (num as f32) / 2.0;

    let cuboid = ShapeHandle::new(Cuboid::new(Vector2::repeat(rad)));

    for i in 0usize..num {
        for j in i..num {
            let fj = j as f32;
            let fi = i as f32;
            let x = (fi * shift / 2.0)
                + (fj - fi) * 2.0 * (rad + ColliderDesc::<f32>::default_margin())
                - centerx;
            let y = fi * 2.0 * (rad + ColliderDesc::<f32>::default_margin()) + rad + 2.0;

            // Build the rigid body.
            let rb = RigidBodyDesc::new().translation(Vector2::new(x, y)).build();
            let rb_handle = bodies.insert(rb);

            // Build the collider.
            let co = ColliderDesc::new(cuboid.clone())
                .density(1.0)
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
    testbed.look_at(Point2::new(0.0, 3.0), 100.0);
}

fn main() {
    let testbed = Testbed::from_builders(0, vec![("FEM surface", init_world)]);
    testbed.run()
}
