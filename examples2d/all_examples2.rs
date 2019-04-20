#![allow(dead_code)]

extern crate nalgebra as na;
extern crate ncollide2d;
extern crate nphysics2d;
extern crate nphysics_testbed2d;

use nphysics_testbed2d::Testbed;

mod balls2;
mod boxes2;
mod capsules2;
mod collision_groups2;
mod compound2;
mod constraints2;
mod convex2;
mod conveyor_belt2;
mod cross2;
mod fem_surface2;
mod force_generator2;
mod heightfield2;
mod kinematic2;
mod mass_constraint_system2;
mod mass_spring_system2;
mod multibody2;
mod plasticity2;
mod ragdoll2;
mod sensor2;
mod polyline2;

fn main() {
    let testbed = Testbed::from_builders(0, vec![
        ("Balls", balls2::init_world),
        ("Boxes", boxes2::init_world),
        ("Capsules", capsules2::init_world),
        ("Collision Groups", collision_groups2::init_world),
        ("CompoundShapes", cross2::init_world),
//        ("Compound Shapes", compound2::init_world),
        ("Constraints", constraints2::init_world),
        ("Convex Polygons", convex2::init_world),
        ("Conveyor Belt", conveyor_belt2::init_world),
        ("FEM Surface", fem_surface2::init_world),
        ("Force Generator", force_generator2::init_world),
        ("Heightfield", heightfield2::init_world),
        ("Kinematic body", kinematic2::init_world),
        ("Mass-constraint System", mass_constraint_system2::init_world),
        ("Mass-spring System", mass_spring_system2::init_world),
        ("Multibody", multibody2::init_world),
        ("Plasticity", plasticity2::init_world),
        ("Ragdolls", ragdoll2::init_world),
        ("Sensor", sensor2::init_world),
        ("Polygonal Line", polyline2::init_world),
    ]);

    testbed.run()
}
