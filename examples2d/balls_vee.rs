extern crate num;
extern crate nalgebra as na;
extern crate ncollide;
extern crate nphysics2d;
extern crate nphysics_testbed2d;

use num::Float;
use na::{Vec2, Translation};
use ncollide::shape::{Ball, Plane};
use nphysics2d::world::World;
use nphysics2d::object::RigidBody;
use nphysics_testbed2d::Testbed;

fn main() {
    let mut world = create_the_world();
    create_the_walls(&mut world);
    create_the_balls(&mut world);
    run_simulation(world);
}


fn create_the_world() -> World<f32> {
    let mut world = World::new();
    world.set_gravity(Vec2::new(0.0, 9.81));
    world
}


fn create_the_walls(world: &mut World<f32>) {
    /*
     * First plane
     */
    let mut rb = RigidBody::new_static(Plane::new(Vec2::new(-1.0, -1.0)), 0.3, 0.6);
    rb.append_translation(&Vec2::new(0.0, 10.0));
    world.add_body(rb);

    /*
     * Second plane
     */
    let mut rb = RigidBody::new_static(Plane::new(Vec2::new(1.0, -1.0)), 0.3, 0.6);
    rb.append_translation(&Vec2::new(0.0, 10.0));
    world.add_body(rb);
}


fn create_the_balls(world: &mut World<f32>) {
    let num     = (4000.0f32.sqrt()) as usize;
    let rad     = 0.5;
    let shift   = 2.5 * rad;
    let centerx = shift * (num as f32) / 2.0;
    let centery = shift * (num as f32) / 2.0;

    for i in 0usize .. num {
        for j in 0usize .. num {
            let x = i as f32 * 2.5 * rad - centerx;
            let y = j as f32 * 2.5 * rad - centery * 2.0 - 20.0;

            let mut rb = RigidBody::new_dynamic(Ball::new(rad), 1.0, 0.3, 0.6);

            rb.append_translation(&Vec2::new(x, y));

            world.add_body(rb);
        }
    }
}


fn run_simulation(world: World<f32>) {
    let mut testbed = Testbed::new(world);

    testbed.run();
}
