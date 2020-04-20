extern crate nalgebra as na;

use na::{Isometry2, Point2, Point3, Vector2};
use ncollide2d::shape::{Cuboid, ShapeHandle};
use nphysics2d::force_generator::DefaultForceGeneratorSet;
use nphysics2d::joint::DefaultJointConstraintSet;
use nphysics2d::object::{
    BodyPartHandle, BodyStatus, ColliderDesc, DefaultBodySet, DefaultColliderSet, FEMSurfaceDesc,
    RigidBodyDesc,
};
use nphysics2d::world::{DefaultGeometricalWorld, DefaultMechanicalWorld};
use nphysics_testbed2d::{r, Real, Testbed};

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mechanical_world = DefaultMechanicalWorld::new(Vector2::zeros());
    let geometrical_world = DefaultGeometricalWorld::new();
    let mut bodies = DefaultBodySet::new();
    let mut colliders = DefaultColliderSet::new();
    let joint_constraints = DefaultJointConstraintSet::new();
    let force_generators = DefaultForceGeneratorSet::new();

    /*
     * Ground.
     */
    let platform_height = 0.4;
    let platform_shape = ShapeHandle::new(Cuboid::new(Vector2::new(0.03, 0.03)));

    let positions = [
        Isometry2::new(Vector2::new(0.4, platform_height), na::zero()),
        Isometry2::new(Vector2::new(0.2, -platform_height), na::zero()),
        Isometry2::new(Vector2::new(0.0, platform_height), na::zero()),
        Isometry2::new(Vector2::new(-0.2, -platform_height), na::zero()),
        Isometry2::new(Vector2::new(-0.4, platform_height), na::zero()),
    ];

    let mut platforms = Vec::new();

    for (i, pos) in positions.iter().enumerate() {
        let platform = RigidBodyDesc::new()
            .position(*pos)
            .status(BodyStatus::Kinematic)
            .build();
        platforms.push(bodies.insert(platform));

        let co = ColliderDesc::new(platform_shape.clone()).build(BodyPartHandle(platforms[i], 0));
        colliders.insert(co);
    }

    /*
     * Create the deformable body and a collider for its contour.
     */
    let mut deformable = FEMSurfaceDesc::quad(20, 1)
        .scale(Vector2::new(1.1, 0.1))
        .density(r!(1.0))
        .young_modulus(1.0e2)
        .mass_damping(0.2)
        .plasticity(0.1, 5.0, 10.0)
        .build();
    let collider_desc = deformable.boundary_collider_desc();
    let deformable_surface_handle = bodies.insert(deformable);
    let co = collider_desc.build(deformable_surface_handle);
    colliders.insert(co);

    /*
     * Set up the testbed.
     */
    testbed.set_world(
        mechanical_world,
        geometrical_world,
        bodies,
        colliders,
        joint_constraints,
        force_generators,
    );
    testbed.set_body_color(deformable_surface_handle, Point3::new(0.0, 0.0, 1.0));

    for platform in &platforms {
        testbed.set_body_color(*platform, Point3::new(0.5, 0.5, 0.5));
    }

    testbed.add_callback(move |_, _, bodies, _, _, _| {
        for (i, handle) in platforms.iter().enumerate() {
            let platform = bodies.rigid_body_mut(*handle).unwrap();
            let platform_y = platform.position().translation.vector.y;

            let mut vel = *platform.velocity();
            let vel_magnitude = 0.1;
            let max_traversal = 0.005;

            if i % 2 == 0 {
                if platform_y <= -max_traversal {
                    vel.linear.y = vel_magnitude;
                } else if platform_y >= platform_height {
                    vel.linear.y = -vel_magnitude;
                }
            } else {
                if platform_y >= max_traversal {
                    vel.linear.y = -vel_magnitude;
                } else if platform_y <= -platform_height {
                    vel.linear.y = vel_magnitude;
                }
            }

            platform.set_velocity(vel);
        }
    });

    testbed.look_at(Point2::origin(), 1000.0);
}

fn main() {
    let testbed = Testbed::from_builders(0, vec![("Plasticity", init_world)]);
    testbed.run()
}
