#![allow(dead_code)]

extern crate nalgebra as na;
extern crate ncollide3d;
extern crate nphysics3d;
extern crate nphysics_testbed3d;

use nphysics_testbed3d::Testbed;

mod balls3;
mod boxes3;
mod capsules3;
mod collision_groups3;
mod compound3;
mod constraints3;
mod convex3;
mod conveyor_belt3;
mod cross3;
mod dzhanibekov3;
mod fem_volume3;
mod force_generator3;
mod heightfield3;
mod kinematic3;
mod mass_constraint_system3;
mod mass_spring_system3;
mod multibody3;
mod plasticity3;
mod ragdoll3;
mod sensor3;
mod trimesh3;

fn main() {
    let testbed = Testbed::from_builders(0, vec![
        ("Balls", balls3::init_world),
        ("Boxes", boxes3::init_world),
        ("Capsules", capsules3::init_world),
        ("Collision Groups", collision_groups3::init_world),
        ("CompoundShapes", cross3::init_world),
//        ("Compound Shapes", compound3::init_world),
        ("Constraints", constraints3::init_world),
        ("Convex Polyhedra", convex3::init_world),
        ("Conveyor Belt", conveyor_belt3::init_world),
        ("Dzhanibekov Effect", dzhanibekov3::init_world),
        ("FEM Volume", fem_volume3::init_world),
        ("Force Generator", force_generator3::init_world),
        ("Heightfield", heightfield3::init_world),
        ("Kinematic body", kinematic3::init_world),
        ("Mass-constraint System", mass_constraint_system3::init_world),
        ("Mass-spring System", mass_spring_system3::init_world),
        ("Multibody", multibody3::init_world),
        ("Plasticity", plasticity3::init_world),
        ("Ragdolls", ragdoll3::init_world),
        ("Sensor", sensor3::init_world),
        ("Triangle Mesh", trimesh3::init_world),
    ]);

    testbed.run()
}
