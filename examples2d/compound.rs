extern crate native;
extern crate "nalgebra" as na;
extern crate ncollide;
extern crate nphysics;
extern crate nphysics_testbed2d;

use std::sync::Arc;
use na::{Vec2, Iso2, Translation};
use ncollide::volumetric::Volumetric;
use ncollide::shape::{Plane, Cuboid, Compound, CompoundData, Shape2};
use nphysics::world::World;
use nphysics::object::RigidBody;
use nphysics_testbed2d::Testbed;

#[start]
fn start(argc: int, argv: *const *const u8) -> int {
    native::start(argc, argv, main)
}

fn main() {
    /*
     * World
     */
    let mut world = World::new();
    world.set_gravity(Vec2::new(0.0, 9.81));

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

    /*
     * Cross shaped geometry
     */
    let delta1 = Iso2::new(Vec2::new(0.0, -5.0), na::zero());
    let delta2 = Iso2::new(Vec2::new(-5.0, 0.0), na::zero());
    let delta3 = Iso2::new(Vec2::new(5.0,  0.0), na::zero());

    let mut cross_geoms = CompoundData::new();
    cross_geoms.push_geom(delta1, Cuboid::new(Vec2::new(4.96, 0.71)), 1.0);
    cross_geoms.push_geom(delta2, Cuboid::new(Vec2::new(0.71, 4.96)), 1.0);
    cross_geoms.push_geom(delta3, Cuboid::new(Vec2::new(0.71, 4.96)), 1.0);

    let compound = Compound::new(cross_geoms);
    let mass     = compound.mass_properties(1.0);
    let cross    = Arc::new(box compound as Box<Shape2<f32>>);

    /*
     * Create the boxes
     */
    let num     = (750.0f32.sqrt()) as uint;
    let rad     = 5.0;
    let shift   = 2.5 * rad;
    let centerx = shift * (num as f32) / 2.0;
    let centery = shift * (num as f32) / 2.0;

    for i in range(0u, num) {
        for j in range(0u, num) {
            let x = i as f32 * 2.5 * rad - centerx;
            let y = j as f32 * 2.5 * rad - centery * 2.0 - 250.0;

            let mut rb = RigidBody::new(cross.clone(), Some(mass), 0.3, 0.6);

            rb.append_translation(&Vec2::new(x, y));

            world.add_body(rb);
        }
    }

    /*
     * Run the simulation.
     */
    let mut testbed = Testbed::new(world);

    testbed.run();
}
