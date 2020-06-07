extern crate nalgebra as na;

use na::{Isometry2, Point2, Point3, RealField, Vector2};
use ncollide2d::shape::{Cuboid, Polyline, ShapeHandle};
use nphysics2d::force_generator::DefaultForceGeneratorSet;
use nphysics2d::joint::DefaultJointConstraintSet;
use nphysics2d::object::{
    BodyPartHandle, ColliderDesc, DefaultBodySet, DefaultColliderSet, DeformableColliderDesc,
    Ground, MassConstraintSystemDesc, RigidBodyDesc,
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
    let polyline = Polyline::quad(50, 1)
        .scaled(&Vector2::new(r!(10.0), r!(1.0)))
        .transformed(&Isometry2::translation(r!(0.0), r!(1.0)));

    let mut deformable = MassConstraintSystemDesc::from_polyline(&polyline)
        .stiffness(Some(r!(1.0e4)))
        .build();

    // Add other constraints for volume stiffness.
    deformable.generate_neighbor_constraints(Some(r!(1.0e4)));
    deformable.generate_neighbor_constraints(Some(r!(1.0e4)));

    let nnodes = deformable.num_nodes();
    let extra_constraints1 = (0..)
        .map(|i| Point2::new(i, nnodes - i - 2))
        .take(nnodes / 2);
    let extra_constraints2 = (1..).map(|i| Point2::new(i, nnodes - i)).take(nnodes / 2);

    for constraint in extra_constraints1.chain(extra_constraints2) {
        deformable.add_constraint(constraint.x, constraint.y, Some(r!(1.0e4)));
    }

    let deformable_handle = bodies.insert(deformable);

    // Collider for the deformable body.
    let deformable_collider =
        DeformableColliderDesc::new(ShapeHandle::new(polyline)).build(deformable_handle);
    colliders.insert(deformable_collider);

    /*
     * Create a pyramid on top of the deformable body.
     */
    let num = 20;
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
                .density(r!(0.1))
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
    testbed.set_body_color(deformable_handle, Point3::new(0.0, 0.0, 1.0));
    testbed.look_at(Point2::new(0.0, 3.0), 100.0);
}

fn main() {
    let testbed = Testbed::<f32>::from_builders(0, vec![("Mass-constraint system", init_world)]);
    testbed.run()
}
