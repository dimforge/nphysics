extern crate nalgebra as na;

use na::{Point2, RealField, Vector2};
use ncollide2d::shape::{Cuboid, ShapeHandle};
use nphysics2d::force_generator::DefaultForceGeneratorSet;
use nphysics2d::joint::DefaultJointConstraintSet;
use nphysics2d::object::{
    BodyPartHandle, ColliderDesc, DefaultBodySet, DefaultColliderSet, FEMSurfaceDesc, Ground,
    RigidBodyDesc,
};
use nphysics2d::world::{DefaultGeometricalWorld, DefaultMechanicalWorld};
use nphysics_testbed2d::Testbed;

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
     * Ground.
     */
    // Ground body shared to which both obstacle colliders will be attached.
    let ground_handle = bodies.insert(Ground::new());

    let obstacle = ShapeHandle::new(Cuboid::new(Vector2::repeat(r!(0.2))));
    let mut obstacle_desc = ColliderDesc::new(obstacle);

    let co = obstacle_desc
        .set_translation(Vector2::x() * r!(4.0))
        .build(BodyPartHandle(ground_handle, 0));
    colliders.insert(co);

    let co = obstacle_desc
        .set_translation(Vector2::x() * r!(-4.0))
        .build(BodyPartHandle(ground_handle, 0));
    colliders.insert(co);

    /*
     * Create the deformable body and a collider for its boundary.
     */
    let mut deformable = FEMSurfaceDesc::quad(50, 1)
        .scale(Vector2::new(r!(10.0), r!(1.0)))
        .translation(Vector2::y() * r!(1.0))
        .young_modulus(r!(1.0e4))
        .mass_damping(r!(0.2))
        .build();
    let collider_desc = deformable.boundary_collider_desc();
    let deformable_handle = bodies.insert(deformable);

    let co = collider_desc.build(deformable_handle);
    colliders.insert(co);

    /*
     * Create a pyramid on top of the deformable body.
     */
    let num = 10;
    let rad = r!(0.1);
    let shift = r!(2.0) * rad;
    let centerx = shift * r!(num as f64) / r!(2.0);

    let cuboid = ShapeHandle::new(Cuboid::new(Vector2::repeat(rad)));

    for i in 0usize..num {
        for j in i..num {
            let fj = r!(j as f64);
            let fi = r!(i as f64);
            let x = (fi * shift / r!(2.0))
                + (fj - fi) * r!(2.0) * (rad + ColliderDesc::<N>::default_margin())
                - centerx;
            let y = fi * r!(2.0) * (rad + ColliderDesc::<N>::default_margin()) + rad + r!(2.0);

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
    testbed.look_at(Point2::new(0.0, 3.0), 100.0);
}

fn main() {
    let testbed = Testbed::<f32>::from_builders(0, vec![("FEM surface", init_world)]);
    testbed.run()
}
