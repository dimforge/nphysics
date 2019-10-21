extern crate nalgebra as na;

use na::{Isometry2, Point2, Vector2};
use ncollide2d::shape::{Cuboid, ShapeHandle};
use nphysics2d::force_generator::DefaultForceGeneratorSet;
use nphysics2d::joint::DefaultJointConstraintSet;
use nphysics2d::object::{
    BodyPartHandle, ColliderDesc, DefaultBodySet, DefaultColliderSet, Ground, RigidBodyDesc,
};
use nphysics2d::world::{DefaultGeometricalWorld, DefaultMechanicalWorld};
use nphysics_testbed2d::Testbed;
use salva2d::boundary::Boundary;
use salva2d::coupling::{ColliderCouplingManager, CouplingMethod};
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
    let particle_rad = 0.01;
    let mut liquid_world = LiquidWorld::new(particle_rad, 1.5);
    let mut coupling_manager = ColliderCouplingManager::new();

    // Liquid.
    let mut points = Vec::new();
    let ni = 30;
    let nj = 50;

    for i in 0..ni {
        for j in 0..nj {
            let x = (i as f32) * particle_rad * 2.0 - ni as f32 * particle_rad;
            let y = (j as f32 + 1.0) * particle_rad * 2.0;
            points.push(Point2::new(x, y));
        }
    }

    let fluid = Fluid::new(points, particle_rad, 1.0, 0.0);
    liquid_world.add_fluid(fluid);

    //    // Ground
    //    let mut points = Vec::new();
    //    let m = 50;
    //    let shift = m as f32 * particle_rad;
    //    for i in 0..m {
    //        let xy = (i as f32) * particle_rad * 2.0 - shift;
    //        points.push(Point2::new(xy, -0.3));
    //        points.push(Point2::new(-shift, shift + xy - 0.3));
    //        points.push(Point2::new(shift, shift + xy - 0.3));
    //    }
    //
    //    let boundary = Boundary::new(points);
    //    liquid_world.add_boundary(boundary);

    // Ground cuboid.
    let ground_size = 25.0;
    let ground_shape = ShapeHandle::new(Cuboid::new(Vector2::new(ground_size, 1.0)));
    let mut ground_particles = Vec::new();
    ground_shape
        .as_surface_sampling()
        .unwrap()
        .sample_surface(particle_rad * 2.0, &mut ground_particles);

    let ground_handle = bodies.insert(Ground::new());
    let co = ColliderDesc::new(ground_shape.clone())
        .translation(-Vector2::y())
        .build(BodyPartHandle(ground_handle, 0));
    let co_handle = colliders.insert(co);
    let bo_handle = liquid_world.add_boundary(Boundary::new(Vec::new()));
    coupling_manager.register_coupling(
        bo_handle,
        co_handle,
        CouplingMethod::DynamicContactSampling,
    );

    let co = ColliderDesc::new(ground_shape.clone())
        .position(Isometry2::new(
            Vector2::x() * -5.0,
            std::f32::consts::FRAC_PI_2 + 0.1,
        ))
        .build(BodyPartHandle(ground_handle, 0));
    let co_handle = colliders.insert(co);
    let bo_handle = liquid_world.add_boundary(Boundary::new(Vec::new()));
    coupling_manager.register_coupling(
        bo_handle,
        co_handle,
        CouplingMethod::DynamicContactSampling,
    );

    let co = ColliderDesc::new(ground_shape)
        .position(Isometry2::new(
            Vector2::x() * 5.0,
            std::f32::consts::FRAC_PI_2 - 0.1,
        ))
        .build(BodyPartHandle(ground_handle, 0));
    let co_handle = colliders.insert(co);
    let bo_handle = liquid_world.add_boundary(Boundary::new(Vec::new()));
    coupling_manager.register_coupling(
        bo_handle,
        co_handle,
        CouplingMethod::DynamicContactSampling,
    );

    // A dynamic cube to play with.
    //    let rb = RigidBodyDesc::new()
    //        .translation(Vector2::y() * 15.0)
    //        .build();
    //    let rb_handle = bodies.insert(rb);
    //    let cube_shape = ShapeHandle::new(Cuboid::new(Vector2::repeat(0.4)));
    //    let mut cube_particles = Vec::new();
    //    cube_shape
    //        .as_surface_sampling()
    //        .unwrap()
    //        .sample_surface(rad, &mut cube_particles);
    //    let co = ColliderDesc::new(cube_shape)
    //        .density(0.5)
    //        .build(BodyPartHandle(rb_handle, 0));
    //    let co_handle = colliders.insert(co);
    //    let bo_handle = liquid_world.add_boundary(Boundary::new(Vec::new()));
    //    coupling_manager.register_coupling(bo_handle, co_handle, cube_particles);

    /*
     * Create a pyramid.
     */
    let num = 5;
    let rad = 0.4;

    let cuboid = ShapeHandle::new(Cuboid::new(Vector2::repeat(rad)));

    let shift = 2.0 * (rad + ColliderDesc::<f32>::default_margin());
    let centerx = shift * (num as f32) / 2.0;
    let centery = rad + ColliderDesc::<f32>::default_margin() * 2.0 + 10.0;

    for i in 0usize..num {
        for j in i..num {
            let fj = j as f32;
            let fi = i as f32;
            let x = (fi * shift / 2.0) + (fj - fi) * shift - centerx;
            let y = fi * shift + centery;

            // Build the rigid body.
            let rb = RigidBodyDesc::new().translation(Vector2::new(x, y)).build();
            let rb_handle = bodies.insert(rb);

            let mut cube_particles = Vec::new();
            cuboid
                .as_surface_sampling()
                .unwrap()
                .sample_surface(particle_rad, &mut cube_particles);

            // Build the collider.
            let co = ColliderDesc::new(cuboid.clone())
                .density(0.5)
                .build(BodyPartHandle(rb_handle, 0));
            let co_handle = colliders.insert(co);
            //            let bo_handle = liquid_world.add_boundary(Boundary::new(Vec::new()));
            //            coupling_manager.register_coupling(
            //                bo_handle,
            //                co_handle,
            //                CouplingMethod::DynamicContactSampling,
            //            );
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
    testbed.set_liquid_world(liquid_world, coupling_manager);
    testbed.look_at(Point2::new(0.0, -2.5), 95.0);
}

fn main() {
    let testbed = Testbed::from_builders(0, vec![("Boxes", init_world)]);
    testbed.run()
}
