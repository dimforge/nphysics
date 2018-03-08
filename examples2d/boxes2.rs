extern crate nalgebra as na;
extern crate ncollide;
extern crate nphysics2d;
extern crate nphysics_testbed2d;

use na::{Isometry2, Vector2};
use ncollide::shape::{Cuboid, ShapeHandle};
use nphysics2d::world::World;
use nphysics2d::object::BodyHandle;
use nphysics2d::volumetric::Volumetric;
use nphysics_testbed2d::Testbed;

const COLLIDER_MARGIN: f32 = 0.01;

fn main() {
    /*
     * World
     */
    let mut world = World::new();
    world.set_gravity(Vector2::new(0.0, 9.81));
    // world.set_max_velocity_iterations(0);
    // world.set_max_position_iterations(1);
    world.set_erp(0.0);
    world.set_warmstart_factor(1.0);

    /*
     * Plane
     */
    let ground_radius = 50.0; // 0.2;
    let ground_shape = ShapeHandle::new(Cuboid::new(Vector2::new(
        ground_radius - COLLIDER_MARGIN,
        ground_radius - COLLIDER_MARGIN,
    )));

    let ground_pos = Isometry2::new(Vector2::y() * ground_radius, na::zero());
    world.add_collider(
        COLLIDER_MARGIN,
        ground_shape,
        BodyHandle::ground(),
        ground_pos,
    );

    /*
    let wall_radius = 50.0; // 0.2;
    let wall_shape = ShapeHandle::new(Cuboid::new(Vector2::new(
        wall_radius - COLLIDER_MARGIN,
        wall_radius - COLLIDER_MARGIN,
    )));
    let wall_pos = Isometry2::new(Vector2::x() * (wall_radius + 5.0), na::zero());
    world.add_collider(
        COLLIDER_MARGIN,
        wall_shape.clone(),
        BodyHandle::ground(),
        wall_pos,
    );

    let wall_pos = Isometry2::new(Vector2::x() * (-wall_radius - 5.0), na::zero());
    world.add_collider(COLLIDER_MARGIN, wall_shape, BodyHandle::ground(), wall_pos);*/

    /*
     * Create the boxes
     */
    let num = 20;
    let rad = 0.2;
    let shift = rad * 2.0;
    let centerx = shift * (num as f32) / 2.0;
    let centery = shift / 2.0;

    let geom = ShapeHandle::new(Cuboid::new(Vector2::repeat(rad - COLLIDER_MARGIN)));
    let inertia = geom.inertia(1.0);

    for i in 0usize..num {
        for j in 0..num {
            let x = i as f32 * shift - centerx;
            let y = -(j as f32 * shift + centery);

            /*
             * Create the rigid body.
             */
            let pos = Isometry2::new(Vector2::new(x, y), na::zero());
            let handle = world.add_rigid_body(pos, inertia);

            /*
             * Create the collider.
             */
            world.add_collider(COLLIDER_MARGIN, geom.clone(), handle, Isometry2::identity());
        }
    }

    /*
     * Set up the testbed.
     */
    let mut testbed = Testbed::new(world);
    testbed.run();
}
