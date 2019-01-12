extern crate nalgebra as na;
extern crate ncollide3d;
extern crate nphysics3d;
extern crate nphysics_testbed3d;

use std::sync::Arc;
use na::{Isometry3, Point3, Vector3};
use ncollide3d::shape::{Cuboid, ShapeHandle, TriMesh};
use nphysics3d::object::{BodyPartHandle, Material, FEMVolume, BodyHandle, BodyStatus, BodyPart};
use nphysics3d::world::World;
use nphysics3d::volumetric::Volumetric;
use nphysics3d::math::Inertia;
use ncollide3d::transformation::ToTriMesh;
use nphysics_testbed3d::Testbed;

const COLLIDER_MARGIN: f32 = 0.01;

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
        ShapeHandle::new(Cuboid::new(Vector3::new(0.03, 0.03, platform_size - COLLIDER_MARGIN)));

    let positions = [
        Isometry3::new(Vector3::new(0.4, platform_height, 0.0), na::zero()),
        Isometry3::new(Vector3::new(0.2, -platform_height, 0.0), na::zero()),
        Isometry3::new(Vector3::new(0.0, platform_height, 0.0), na::zero()),
        Isometry3::new(Vector3::new(-0.2, -platform_height, 0.0), na::zero()),
        Isometry3::new(Vector3::new(-0.4, platform_height, 0.0), na::zero()),
    ];

    let mut platforms = [BodyHandle::ground(); 5];

    for (i, pos) in positions.iter().enumerate() {
        // NOTE: it is OK to set an inertia equal to zero for a body that will only be kinematic.
        let part = world.add_rigid_body(*pos, Inertia::zero(), Point3::origin());
        world.body_mut(part.0).set_status(BodyStatus::Kinematic);

        world.add_collider(
            COLLIDER_MARGIN,
            platform_shape.clone(),
            part,
            Isometry3::identity(),
            Material::default(),
        );

        platforms[i] = part.0;
    }

    /*
     * Create the deformable body and a collider for its contour.
     */
    let mut volume = FEMVolume::cube(
        &Isometry3::identity(),
        &Vector3::new(1.1, 0.1, 0.1),
        50, 1, 1,
        1.0, 1.0e3, 0.0,
        (0.2, 0.0));
    let (mesh, ids_map, parts_map) = volume.boundary_mesh();
    volume.renumber_dofs(&ids_map);
    volume.set_plasticity(0.1, 20.0, 1.0e5);

    let handle = world.add_body(Box::new(volume));
    world.add_deformable_collider(
        COLLIDER_MARGIN,
        mesh,
        handle,
        None,
        Some(Arc::new(parts_map)),
        Material::default(),
    );
    world.body_mut(handle).set_deactivation_threshold(None);

    /*
     * Set up the testbed.
     */
    let mut testbed = Testbed::new(world);

    for platform in &platforms {
        testbed.set_body_color(*platform, Point3::new(0.5, 0.5, 0.5));
    }


    testbed.add_callback(move |world, _, time| {
        for (i, handle) in platforms.iter().enumerate() {
            let platform = world.rigid_body_mut(*handle).unwrap();
            let platform_y = platform.position().translation.vector.y;

            let mut vel = platform.velocity();
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

    // testbed.hide_performance_counters();
    testbed.look_at(Point3::new(0.0, 0.0, 2.0), Point3::new(0.0, 0.0, 0.0));
    testbed.run();
}
