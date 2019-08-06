extern crate nalgebra as na;

use na::{Point2, Vector2};
use ncollide2d::shape::{Polyline, Cuboid, ShapeHandle};
use nphysics2d::object::{ColliderDesc, RigidBodyDesc, DefaultBodySet, DefaultColliderSet, Ground, BodyPartHandle};
use nphysics2d::force_generator::DefaultForceGeneratorSet;
use nphysics2d::joint::DefaultJointConstraintSet;
use nphysics2d::world::{DefaultMechanicalWorld, DefaultGeometricalWorld};
use nphysics_testbed2d::Testbed;


use rand::distributions::{Standard, Distribution};
use rand::{SeedableRng, rngs::StdRng};


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
    let num_split = 20;
    let begin = -15.0f32;
    let max_h = 3.0;
    let begin_h = -3.0;
    let step = (begin.abs() * 2.0) / (num_split as f32);
    let mut vertices: Vec<Point2<f32>> = (0..num_split + 2)
        .map(|i| Point2::new(begin + (i as f32) * step, 0.0))
        .collect();

    let mut rng = StdRng::seed_from_u64(0);
    let distribution = Standard;

    for i in 0usize..num_split {
        let h: f32 = distribution.sample(&mut rng);
        vertices[i + 1].y = h * max_h + begin_h;
    }

    let polyline = ShapeHandle::new(Polyline::new(vertices, None));
    let ground_handle = bodies.insert(Ground::new());
    let co = ColliderDesc::new(polyline)
        .translation(-Vector2::y())
        .build(BodyPartHandle(ground_handle, 0));
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
            let y = fi * shift + 0.5;

            // Build the rigid body.
            let rb = RigidBodyDesc::new()
                .translation(Vector2::new(x, y))
                .build();
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
    testbed.set_world(mechanical_world, geometrical_world, bodies, colliders, joint_constraints, force_generators);
    testbed.look_at(Point2::origin(), 75.0);
}


fn main() {
    let testbed = Testbed::from_builders(0, vec![
        ("Polyline", init_world),
    ]);
    testbed.run();
}