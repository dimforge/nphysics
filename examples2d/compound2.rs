extern crate nalgebra as na;
extern crate ncollide2d;
extern crate nphysics2d;
extern crate nphysics_testbed2d;

use na::{Isometry2, Vector2};
use ncollide2d::shape::{Compound, Cuboid, ShapeHandle};
use nphysics2d::object::{BodyHandle, Material};
use nphysics2d::volumetric::Volumetric;
use nphysics2d::world::World;
use nphysics_testbed2d::Testbed;

const COLLIDER_MARGIN: f32 = 0.01;

fn main() {
    /*
     * World
     */
    let mut world = World::new();
    world.set_gravity(Vector2::new(0.0, -9.81));

    /*
     * Ground
     */
    let ground_radx = 25.0;
    let ground_rady = 1.0;
    let ground_shape = ShapeHandle::new(Cuboid::new(Vector2::new(
        ground_radx - COLLIDER_MARGIN,
        ground_rady - COLLIDER_MARGIN,
    )));

    let ground_pos = Isometry2::new(Vector2::y() * ground_rady, na::zero());
    world.add_collider(
        COLLIDER_MARGIN,
        ground_shape,
        BodyHandle::ground(),
        ground_pos,
        Material::default(),
    );

    /*
     * Cross shaped geometry
     */
    let large_rad = 1.0f32;
    let small_rad = 0.05f32;

    let delta1 = Isometry2::new(Vector2::new(0.0, large_rad), na::zero());
    let delta2 = Isometry2::new(Vector2::new(-large_rad, 0.0), na::zero());
    let delta3 = Isometry2::new(Vector2::new(large_rad, 0.0), na::zero());

    let mut cross_geoms = Vec::new();
    let vertical = ShapeHandle::new(Cuboid::new(Vector2::new(
        small_rad - COLLIDER_MARGIN,
        large_rad - COLLIDER_MARGIN,
    )));
    let horizontal = ShapeHandle::new(Cuboid::new(Vector2::new(
        large_rad - COLLIDER_MARGIN,
        small_rad - COLLIDER_MARGIN,
    )));
    cross_geoms.push((delta1, horizontal));
    cross_geoms.push((delta2, vertical.clone()));
    cross_geoms.push((delta3, vertical));

    let compound = Compound::new(cross_geoms);
    let cross = ShapeHandle::new(compound);
    let inertia = cross.inertia(1.0);
    let center_of_mass = cross.center_of_mass();

    /*
     * Create the boxes
     */
    let num = 15;
    let shift = 2.5 * large_rad;
    let centerx = shift * (num as f32) / 2.0;
    let centery = shift * (num as f32) / 2.0;

    for i in 0usize..num {
        for j in 0usize..num {
            let x = i as f32 * 2.5 * large_rad - centerx;
            let y = j as f32 * 2.5 * -large_rad + centery * 2.0;

            /*
             * Create the rigid body.
             */
            let pos = Isometry2::new(Vector2::new(x, y), na::zero());
            let handle = world.add_rigid_body(pos, inertia, center_of_mass);

            /*
             * Create the collider.
             */
            world.add_collider(
                COLLIDER_MARGIN,
                cross.clone(),
                handle,
                Isometry2::identity(),
                Material::default(),
            );
        }
    }

    /*
     * Run the simulation.
     */
    let testbed = Testbed::new(world);
    testbed.run();
}
