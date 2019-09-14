extern crate nalgebra as na;

use na::{DVector, Point2, Vector2};
use ncollide2d::shape::{Cuboid, HeightField, ShapeHandle};
use nphysics2d::force_generator::DefaultForceGeneratorSet;
use nphysics2d::joint::DefaultJointConstraintSet;
use nphysics2d::object::{
    BodyPartHandle, ColliderDesc, DefaultBodySet, DefaultColliderSet, Ground, RigidBodyDesc,
};
use nphysics2d::world::{DefaultGeometricalWorld, DefaultMechanicalWorld};
use nphysics_testbed2d::Testbed;

use rand::{rngs::StdRng, Rng, SeedableRng};

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
     * Polyline
     */
    let mut rng = StdRng::seed_from_u64(0);
    let heights = DVector::from_fn(20, |_, _| rng.gen::<f32>());

    let mut heightfield = HeightField::new(heights, Vector2::new(20.0, 1.0));

    // It is possible to remove some segments from the heightfield.
    heightfield.set_segment_removed(3, true);
    heightfield.set_segment_removed(13, true);

    let ground_handle = bodies.insert(Ground::new());
    let co =
        ColliderDesc::new(ShapeHandle::new(heightfield)).build(BodyPartHandle(ground_handle, 0));
    colliders.insert(co);

    /*
     * Create the boxes
     */
    let width = 75;
    let height = 7;
    let rad = 0.1;

    let cuboid = ShapeHandle::new(Cuboid::new(Vector2::repeat(rad)));

    let shift = 2.0 * (rad + ColliderDesc::<f32>::default_margin());
    let centerx = shift * (width as f32) / 2.0;

    for i in 0usize..height {
        for j in 0usize..width {
            let fj = j as f32;
            let fi = i as f32;
            let x = fj * shift - centerx;
            let y = fi * shift + 1.0;

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
     * Run the simulation.
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
    testbed.look_at(Point2::origin(), 75.0);
}

fn main() {
    let testbed = Testbed::from_builders(0, vec![("Heightfield", init_world)]);
    testbed.run()
}
