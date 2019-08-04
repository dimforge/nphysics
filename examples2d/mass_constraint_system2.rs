extern crate nalgebra as na;

use na::{Point2, Vector2, Point3, Isometry2};
use ncollide2d::shape::{Cuboid, ShapeHandle, Polyline};
use nphysics2d::object::{MassConstraintSystemDesc, ColliderDesc, DeformableColliderDesc, RigidBodyDesc,
                         DefaultBodySet, DefaultColliderSet, Ground, BodyPartHandle};
use nphysics2d::force_generator::DefaultForceGeneratorSet;
use nphysics2d::joint::DefaultJointConstraintSet;
use nphysics2d::world::{DefaultDynamicWorld, DefaultColliderWorld};
use nphysics_testbed2d::Testbed;


pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let dynamic_world = DefaultDynamicWorld::new(Vector2::new(0.0, -9.81));
    let collider_world = DefaultColliderWorld::new();
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
    let polyline = Polyline::quad(50, 1)
        .scaled(&Vector2::new(10.0, 1.0))
        .transformed(&Isometry2::translation(0.0, 1.0));

    let mut deformable = MassConstraintSystemDesc::from_polyline(&polyline)
        .stiffness(Some(1.0e4))
        .build();

    // Add other constraints for volume stiffness.
    deformable.generate_neighbor_constraints(Some(1.0e4));
    deformable.generate_neighbor_constraints(Some(1.0e4));

    let nnodes = deformable.num_nodes();
    let extra_constraints1 = (0..).map(|i| Point2::new(i, nnodes - i - 2)).take(nnodes / 2);
    let extra_constraints2 = (1..).map(|i| Point2::new(i, nnodes - i)).take(nnodes / 2);

    for constraint in extra_constraints1.chain(extra_constraints2) {
        deformable.add_constraint(constraint.x, constraint.y, Some(1.0e4));
    }

    let deformable_handle = bodies.insert(deformable);

    // Collider for the deformable body.
    let deformable_collider = DeformableColliderDesc::new(ShapeHandle::new(polyline))
        .build(deformable_handle);
    colliders.insert(deformable_collider);

    /*
     * Create a pyramid on top of the deformable body.
     */
    let num = 20;
    let rad = 0.1;
    let shift = 2.0 * rad;
    let centerx = shift * (num as f32) / 2.0;

    let cuboid = ShapeHandle::new(Cuboid::new(Vector2::repeat(rad)));

    for i in 0usize..num {
        for j in i..num {
            let fj = j as f32;
            let fi = i as f32;
            let x = (fi * shift / 2.0) + (fj - fi) * 2.0 * (rad + ColliderDesc::<f32>::default_margin()) - centerx;
            let y = fi * 2.0 * (rad + ColliderDesc::<f32>::default_margin()) + rad + 2.0;

            // Build the rigid body.
            let rb = RigidBodyDesc::new()
                .translation(Vector2::new(x, y))
                .build();
            let rb_handle = bodies.insert(rb);

            // Build the collider.
            let co = ColliderDesc::new(cuboid.clone())
                .density(0.1)
                .build(BodyPartHandle(rb_handle, 0));
            colliders.insert(co);
        }
    }

    /*
     * Set up the testbed.
     */
    testbed.set_ground_handle(Some(ground_handle));
    testbed.set_world(dynamic_world, collider_world, bodies, colliders, joint_constraints, force_generators);
    testbed.set_body_color(deformable_handle, Point3::new(0.0, 0.0, 1.0));
    testbed.look_at(Point2::new(0.0, -3.0), 100.0);
}


fn main() {
    let testbed = Testbed::from_builders(0, vec![
        ("Mass-constraint system", init_world),
    ]);
    testbed.run()
}