extern crate nalgebra as na;

use na::{Isometry3, Point3, Vector3};
use ncollide3d::shape::{Cuboid, ShapeHandle};
use nphysics3d::force_generator::DefaultForceGeneratorSet;
use nphysics3d::joint::DefaultJointConstraintSet;
use nphysics3d::object::{
    BodyPartHandle, BodyStatus, ColliderDesc, DefaultBodySet, DefaultColliderSet, FEMVolumeDesc,
    Ground, RigidBodyDesc,
};
use nphysics3d::world::{DefaultGeometricalWorld, DefaultMechanicalWorld};
use nphysics_testbed3d::{r, Testbed};

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mechanical_world = DefaultMechanicalWorld::new(Vector3::zeros());
    let geometrical_world = DefaultGeometricalWorld::new();
    let mut bodies = DefaultBodySet::new();
    let mut colliders = DefaultColliderSet::new();
    let joint_constraints = DefaultJointConstraintSet::new();
    let force_generators = DefaultForceGeneratorSet::new();

    /*
     * Ground.
     */
    let platform_height = r!(0.4);
    let platform_size = r!(0.6);
    let platform_shape =
        ShapeHandle::new(Cuboid::new(Vector3::new(r!(0.03), r!(0.03), platform_size)));

    let positions = [
        Isometry3::new(Vector3::new(r!(0.4), platform_height, r!(0.0)), na::zero()),
        Isometry3::new(Vector3::new(r!(0.2), -platform_height, r!(0.0)), na::zero()),
        Isometry3::new(Vector3::new(r!(0.0), platform_height, r!(0.0)), na::zero()),
        Isometry3::new(
            Vector3::new(r!(-0.2), -platform_height, r!(0.0)),
            na::zero(),
        ),
        Isometry3::new(Vector3::new(r!(-0.4), platform_height, r!(0.0)), na::zero()),
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
    let mut deformable = FEMVolumeDesc::cube(20, 1, 1)
        .scale(Vector3::new(r!(1.1), r!(0.1), r!(0.1)))
        .density(r!(1.0))
        .young_modulus(r!(1.0e2))
        .mass_damping(r!(0.2))
        .plasticity(r!(0.1), r!(5.0), r!(10.0))
        .build();
    let collider_desc = deformable.boundary_collider_desc();
    let deformable_surface_handle = bodies.insert(deformable);
    let co = collider_desc.build(deformable_surface_handle);
    colliders.insert(co);

    /*
     * Set up the testbed.
     */
    testbed.set_body_color(deformable_surface_handle, Point3::new(0.0, 0.0, 1.0));

    for platform in &platforms {
        testbed.set_body_color(*platform, Point3::new(0.5, 0.5, 0.5));
    }

    testbed.add_callback(move |_, _, bodies, _, _, _| {
        for (i, handle) in platforms.iter().enumerate() {
            let platform = bodies.rigid_body_mut(*handle).unwrap();
            let platform_y = platform.position().translation.vector.y;

            let mut vel = *platform.velocity();
            let vel_magnitude = r!(0.1);
            let max_traversal = r!(0.005);

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

    // NOTE: we add another static body to the scene. It is not necessary for our simulation
    // but this is required so that we can call `testbed.set_ground_handle` which will
    // enable the testbed's feature that lets us grab an object with the mouse.
    let ground_handle = bodies.insert(Ground::new());
    testbed.set_ground_handle(Some(ground_handle));
    testbed.set_world(
        mechanical_world,
        geometrical_world,
        bodies,
        colliders,
        joint_constraints,
        force_generators,
    );
    testbed.look_at(Point3::new(0.0, 0.0, 2.0), Point3::new(0.0, 0.0, 0.0));
}

fn main() {
    let testbed = Testbed::from_builders(0, vec![("Plasticity", init_world)]);
    testbed.run()
}
