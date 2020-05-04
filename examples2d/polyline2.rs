extern crate nalgebra as na;

use na::{Point2, RealField, Vector2};
use ncollide2d::shape::{Cuboid, Polyline, ShapeHandle};
use nphysics2d::force_generator::DefaultForceGeneratorSet;
use nphysics2d::joint::DefaultJointConstraintSet;
use nphysics2d::object::{
    BodyPartHandle, ColliderDesc, DefaultBodySet, DefaultColliderSet, Ground, RigidBodyDesc,
};
use nphysics2d::world::{DefaultGeometricalWorld, DefaultMechanicalWorld};
use nphysics_testbed2d::Testbed;

use rand::distributions::{Distribution, Standard};
use rand::{rngs::StdRng, SeedableRng};

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
     * Polyline
     */
    let num_split = 20;
    let begin = r!(-15.0);
    let max_h = r!(3.0);
    let begin_h = r!(-3.0);
    let step = (begin.abs() * r!(2.0)) / r!(num_split as f64);
    let mut vertices: Vec<Point2<N>> = (0..num_split + 2)
        .map(|i| Point2::new(begin + r!(i as f64) * step, r!(0.0)))
        .collect();

    let mut rng = StdRng::seed_from_u64(0);
    let distribution = Standard;

    for i in 0usize..num_split {
        let h: f64 = distribution.sample(&mut rng);
        vertices[i + 1].y = r!(h) * max_h + begin_h;
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
    let rad = r!(0.1);

    let cuboid = ShapeHandle::new(Cuboid::new(Vector2::repeat(rad)));

    let shift = r!(2.0) * (rad + ColliderDesc::<N>::default_margin());
    let centerx = shift * r!(width as f64) / r!(2.0);

    for i in 0usize..height {
        for j in 0usize..width {
            let fj = r!(j as f64);
            let fi = r!(i as f64);
            let x = fj * shift - centerx;
            let y = fi * shift + r!(0.5);

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
    let testbed = Testbed::<f32>::from_builders(0, vec![("Polyline", init_world)]);
    testbed.run();
}
