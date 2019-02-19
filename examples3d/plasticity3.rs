extern crate nalgebra as na;
extern crate ncollide3d;
extern crate nphysics3d;
extern crate nphysics_testbed3d;

use na::{Isometry3, Point3, Vector3};
use ncollide3d::shape::{Cuboid, ShapeHandle};
use nphysics3d::object::{ColliderDesc, RigidBodyDesc, FEMVolumeDesc, BodyHandle, BodyStatus};
use nphysics3d::world::World;
use nphysics_testbed3d::Testbed;


fn main() {
    /*
     * World
     */
    let mut world = World::new();

    /*
     * Ground.
     */
    let platform_height = 0.4;
    let platform_size = 0.6;
    let platform_shape =
        ShapeHandle::new(Cuboid::new(Vector3::new(0.03, 0.03, platform_size)));

    let positions = [
        Isometry3::new(Vector3::new(0.4, platform_height, 0.0), na::zero()),
        Isometry3::new(Vector3::new(0.2, -platform_height, 0.0), na::zero()),
        Isometry3::new(Vector3::new(0.0, platform_height, 0.0), na::zero()),
        Isometry3::new(Vector3::new(-0.2, -platform_height, 0.0), na::zero()),
        Isometry3::new(Vector3::new(-0.4, platform_height, 0.0), na::zero()),
    ];

    let mut platforms = [BodyHandle::ground(); 5];

    for (i, pos) in positions.iter().enumerate() {
        platforms[i] = RigidBodyDesc::new()
            .position(*pos)
            .status(BodyStatus::Kinematic)
            .collider(&ColliderDesc::new(platform_shape.clone()))
            .build(&mut world)
            .handle();
    }

    /*
     * Create the deformable body and a collider for its contour.
     */
    FEMVolumeDesc::cube(20, 1, 1)
        .scale(Vector3::new(1.1, 0.1, 0.1))
        .density(1.0)
        .young_modulus(1.0e2)
        .mass_damping(0.2)
        .plasticity(0.1, 5.0, 10.0)
        .collider_enabled(true)
        .build(&mut world);

    /*
     * Set up the testbed.
     */
    let mut testbed = Testbed::new(world);

    for platform in &platforms {
        testbed.set_body_color(*platform, Point3::new(0.5, 0.5, 0.5));
    }


    testbed.add_callback(move |world, _, _| {
        let mut world = world.get_mut();
        for (i, handle) in platforms.iter().enumerate() {
            let platform = world.rigid_body_mut(*handle).unwrap();
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

    testbed.look_at(Point3::new(0.0, 0.0, 2.0), Point3::new(0.0, 0.0, 0.0));
    testbed.show_performance_counters();
    testbed.run();
}
