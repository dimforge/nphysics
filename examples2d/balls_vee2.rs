extern crate nalgebra as na;
extern crate ncollide2d;
extern crate nphysics2d;
extern crate nphysics_testbed2d;

use na::{Isometry2, Unit, Vector2};
use ncollide2d::shape::{Ball, Plane, ShapeHandle};
use nphysics2d::object::{BodyHandle, Material};
use nphysics2d::volumetric::Volumetric;
use nphysics2d::world::World;
use nphysics_testbed2d::Testbed;

const COLLIDER_MARGIN: f32 = 0.01;

fn main() {
    let mut world = create_the_world();
    create_the_walls(&mut world);
    create_the_balls(&mut world);
    run_simulation(world);
}

fn create_the_world() -> World<f32> {
    let mut world = World::new();
    world.set_gravity(Vector2::new(0.0, -9.81));
    world
}

fn create_the_walls(world: &mut World<f32>) {
    /*
     * First plane
     */
    let n = Unit::new_normalize(Vector2::new(-1.0, 1.0));
    let ground_shape = ShapeHandle::new(Plane::new(n));

    world.add_collider(
        COLLIDER_MARGIN,
        ground_shape,
        BodyHandle::ground(),
        Isometry2::new(-Vector2::y() * 10.0, na::zero()),
        Material::default(),
    );

    /*
     * Second plane
     */
    let n = Unit::new_normalize(Vector2::new(1.0, 1.0));
    let ground_shape = ShapeHandle::new(Plane::new(n));

    world.add_collider(
        COLLIDER_MARGIN,
        ground_shape,
        BodyHandle::ground(),
        Isometry2::new(-Vector2::y() * 10.0, na::zero()),
        Material::default(),
    );
}

fn create_the_balls(world: &mut World<f32>) {
    let num = (2000.0f32.sqrt()) as usize;
    let rad = 0.1;
    let shift = 2.5 * rad;
    let centerx = shift * (num as f32) / 2.0;
    let centery = shift * (num as f32) / 2.0;

    let geom = ShapeHandle::new(Ball::new(rad - COLLIDER_MARGIN));
    let inertia = geom.inertia(1.0);
    let center_of_mass = geom.center_of_mass();

    for i in 0usize..num {
        for j in 0usize..num {
            let x = i as f32 * 2.5 * rad - centerx;
            let y = j as f32 * 2.5 * rad + centery * 2.0 + 0.5;

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
                geom.clone(),
                handle,
                Isometry2::identity(),
                Material::default(),
            );
        }
    }
}

fn run_simulation(world: World<f32>) {
    let testbed = Testbed::new(world);
    testbed.run();
}
