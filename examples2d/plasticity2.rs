extern crate nalgebra as na;
extern crate ncollide2d;
extern crate nphysics2d;
extern crate nphysics_testbed2d;

use na::{Isometry2, Point3, Vector2, Point2};
use ncollide2d::shape::{Cuboid, ShapeHandle};
use nphysics2d::object::{RigidBodyDesc, ColliderDesc, FEMSurfaceDesc, BodyHandle, BodyStatus};
use nphysics2d::world::World;
use nphysics_testbed2d::Testbed;


fn main() {
    /*
     * World
     */
    let mut world = World::new();

    /*
     * Ground.
     */
    let platform_height = 0.4;
    let platform_shape =
        ShapeHandle::new(Cuboid::new(Vector2::new(0.03, 0.03)));

    let positions = [
        Isometry2::new(Vector2::new(0.4, platform_height), na::zero()),
        Isometry2::new(Vector2::new(0.2, -platform_height), na::zero()),
        Isometry2::new(Vector2::new(0.0, platform_height), na::zero()),
        Isometry2::new(Vector2::new(-0.2, -platform_height), na::zero()),
        Isometry2::new(Vector2::new(-0.4, platform_height), na::zero()),
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
    let deformable_surface_handle = FEMSurfaceDesc::quad(20, 1)
        .scale(Vector2::new(1.1, 0.1))
        .density(1.0)
        .young_modulus(1.0e2)
        .mass_damping(0.2)
        .plasticity(0.1, 5.0, 10.0)
        .collider_enabled(true)
        .build(&mut world)
        .handle();

    /*
     * Set up the testbed.
     */
    let mut testbed = Testbed::new(world);
    testbed.set_body_color(deformable_surface_handle, Point3::new(0.0, 0.0, 1.0));

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

    testbed.look_at(Point2::origin(), 1000.0);
    testbed.run();
}
