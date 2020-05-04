extern crate nalgebra as na;

use na::{Point3, RealField, Vector3};
use ncollide3d::pipeline::CollisionGroups;
use ncollide3d::shape::{Cuboid, ShapeHandle};
use nphysics3d::force_generator::DefaultForceGeneratorSet;
use nphysics3d::joint::DefaultJointConstraintSet;
use nphysics3d::object::{
    BodyPartHandle, ColliderDesc, DefaultBodySet, DefaultColliderSet, Ground, RigidBodyDesc,
};
use nphysics3d::world::{DefaultGeometricalWorld, DefaultMechanicalWorld};
use nphysics_testbed3d::Testbed;

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
     * Setup a static body used as the ground.
     */
    let ground_handle = bodies.insert(Ground::new());

    /*
     * Setup groups.
     */
    const GREEN_GROUP_ID: usize = 0;
    let mut green_group = CollisionGroups::new();
    green_group.set_membership(&[GREEN_GROUP_ID]);
    green_group.set_whitelist(&[GREEN_GROUP_ID]);

    const BLUE_GROUP_ID: usize = 1;
    let mut blue_group = CollisionGroups::new();
    blue_group.set_membership(&[BLUE_GROUP_ID]);
    blue_group.set_whitelist(&[BLUE_GROUP_ID]);

    /*
     * A floor that will collide with everything (default behaviour).
     */
    let ground_thickness = r!(0.2);
    let ground_shape = ShapeHandle::new(Cuboid::new(Vector3::new(
        r!(3.0),
        ground_thickness,
        r!(3.0),
    )));

    let main_floor = ColliderDesc::new(ground_shape)
        .translation(Vector3::y() * -ground_thickness)
        .build(BodyPartHandle(ground_handle, 0));
    colliders.insert(main_floor);

    /*
     * A green floor that will collide with the GREEN group only.
     */
    let ground_shape = ShapeHandle::new(Cuboid::new(Vector3::new(r!(1.0), r!(0.1), r!(1.0))));
    let green_floor = ColliderDesc::new(ground_shape.clone())
        .translation(Vector3::y())
        .collision_groups(green_group)
        .build(BodyPartHandle(ground_handle, 0));
    let green_collider_handle = colliders.insert(green_floor);

    testbed.set_collider_color(green_collider_handle, Point3::new(0.0, 1.0, 0.0));

    /*
     * A blue floor that will collide with the BLUE group only.
     */
    let blue_floor = ColliderDesc::new(ground_shape)
        .translation(Vector3::y() * r!(2.0))
        .collision_groups(blue_group)
        .build(BodyPartHandle(ground_handle, 0));
    let blue_collider_handle = colliders.insert(blue_floor);

    testbed.set_collider_color(blue_collider_handle, Point3::new(0.0, 0.0, 1.0));

    /*
     * Create the boxes
     */
    let num = 8;
    let rad = r!(0.1);
    let shift = (rad + ColliderDesc::<N>::default_margin()) * r!(2.0);
    let centerx = shift * r!(num as f64) / r!(2.0);
    let centery = r!(2.5);
    let centerz = shift * r!(num as f64) / r!(2.0);

    let cuboid = ShapeHandle::new(Cuboid::new(Vector3::repeat(rad)));

    for k in 0usize..4 {
        for i in 0usize..num {
            for j in 0usize..num {
                let x = r!(i as f64) * shift - centerx;
                let z = r!(j as f64) * shift - centerz;
                let y = r!(k as f64) * shift + centery;

                // Alternate between the GREEN and BLUE groups.
                let (group, color) = if k % 2 == 0 {
                    (green_group, Point3::new(0.0, 1.0, 0.0))
                } else {
                    (blue_group, Point3::new(0.0, 0.0, 1.0))
                };

                // Build the rigid body.
                let rb = RigidBodyDesc::new()
                    .translation(Vector3::new(x, y, z))
                    .build();
                let rb_handle = bodies.insert(rb);

                // Build the collider.
                let co = ColliderDesc::new(cuboid.clone())
                    .density(r!(1.0))
                    .collision_groups(group)
                    .build(BodyPartHandle(rb_handle, 0));
                colliders.insert(co);

                testbed.set_body_color(rb_handle, color);
            }
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
    testbed.look_at(Point3::new(-4.0, 1.0, -4.0), Point3::new(0.0, 1.0, 0.0));
}

fn main() {
    let testbed = Testbed::<f32>::from_builders(0, vec![("Collision groups", init_world)]);
    testbed.run()
}
