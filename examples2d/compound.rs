extern crate num;
extern crate nalgebra as na;
extern crate ncollide;
extern crate nphysics2d;
extern crate nphysics_testbed2d;

use std::sync::Arc;
use num::Float;
use na::{Vec2, Iso2, Translation};
use ncollide::shape::{Plane, Cuboid, Compound};
use ncollide::inspection::Repr2;
use nphysics2d::volumetric::Volumetric;
use nphysics2d::world::World;
use nphysics2d::object::RigidBody;
use nphysics_testbed2d::Testbed;

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

    let mut cross_geoms = Vec::new();
    let vertical   = Arc::new(Box::new(Cuboid::new(Vec2::new(0.21f32, 4.96))) as Box<Repr2<f32>>);
    let horizontal = Arc::new(Box::new(Cuboid::new(Vec2::new(4.96f32, 0.21))) as Box<Repr2<f32>>);
    cross_geoms.push((delta1, horizontal));
    cross_geoms.push((delta2, vertical.clone()));
    cross_geoms.push((delta3, vertical));

    let compound = Compound::new(cross_geoms);
    let mass     = compound.mass_properties(1.0);
    let cross    = Arc::new(Box::new(compound) as Box<Repr2<f32>>);

    /*
     * Create the boxes
     */
    let num     = (750.0f32.sqrt()) as usize;
    let rad     = 5.0;
    let shift   = 2.5 * rad;
    let centerx = shift * (num as f32) / 2.0;
    let centery = shift * (num as f32) / 2.0;

    for i in 0usize .. num {
        for j in 0usize .. num {
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
