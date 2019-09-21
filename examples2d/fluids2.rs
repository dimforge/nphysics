extern crate nalgebra as na;

use na::{Point2, Vector2};
use ncollide2d::shape::{Cuboid, ShapeHandle};
use nphysics2d::force_generator::DefaultForceGeneratorSet;
use nphysics2d::joint::DefaultJointConstraintSet;
use nphysics2d::object::{
    BodyPartHandle, ColliderDesc, DefaultBodySet, DefaultColliderSet, Ground, RigidBodyDesc,
};
use nphysics2d::world::{DefaultGeometricalWorld, DefaultMechanicalWorld};
use nphysics_testbed2d::Testbed;
use salva2d::fluid::Fluid;
use salva2d::LiquidWorld;

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
     * Liquid world.
     */
    let mut liquid_world = LiquidWorld::new(0.1);

    let mut points = Vec::new();
    let n = 100;
    let rad = 0.1;
    let vol = rad * rad * 3.0;

    for i in 0..n {
        for j in 0..n {
            let x = (i as f32) * rad * 2.0;
            let y = (j as f32) * rad * 2.0;
            points.push(Point2::new(x, y));
        }
    }
    let fluid = Fluid::new(points, vol, 1.0);
    liquid_world.add_fluid(fluid);

    /*
     * Set up the testbed.
     */
    //    testbed.set_ground_handle(Some(ground_handle));
    testbed.set_world(
        mechanical_world,
        geometrical_world,
        bodies,
        colliders,
        joint_constraints,
        force_generators,
    );
    testbed.set_liquid_world(liquid_world);
    testbed.look_at(Point2::new(0.0, -2.5), 95.0);
}

fn main() {
    let testbed = Testbed::from_builders(0, vec![("Boxes", init_world)]);
    testbed.run()
}
