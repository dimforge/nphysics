#![allow(dead_code)]

extern crate nalgebra as na;
extern crate ncollide3d;
extern crate nphysics3d;
extern crate nphysics_testbed3d;

macro_rules! r(
    ($e: expr) => {
        nalgebra::convert::<f64, N>($e)
    }
);

use inflector::Inflector;

use nphysics_testbed3d::Testbed;

mod balls3;
mod boxes3;
mod broad_phase_filter3;
mod capsules3;
mod ccd3;
mod collision_groups3;
mod compound3;
mod constraints3;
mod convex3;
mod conveyor_belt3;
mod cross3;
mod damping3;
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

fn demo_name_from_command_line() -> Option<String> {
    let mut args = std::env::args();

    while let Some(arg) = args.next() {
        if &arg[..] == "--example" {
            return args.next();
        }
    }

    None
}

#[cfg(any(target_arch = "wasm32", target_arch = "asmjs"))]
fn demo_name_from_url() -> Option<String> {
    let window = stdweb::web::window();
    let hash = window.location()?.search().ok()?;
    if !hash.is_empty() {
        Some(hash[1..].to_string())
    } else {
        None
    }
}

#[cfg(not(any(target_arch = "wasm32", target_arch = "asmjs")))]
fn demo_name_from_url() -> Option<String> {
    None
}

fn main() {
    type Real = f32; // simba::scalar::FixedI40F24;

    let demo = demo_name_from_command_line()
        .or_else(|| demo_name_from_url())
        .unwrap_or(String::new())
        .to_camel_case();

    let mut builders: Vec<(_, fn(&mut Testbed<Real>))> = vec![
        ("Balls", balls3::init_world),
        ("Boxes", boxes3::init_world),
        ("Broad-phase filter", broad_phase_filter3::init_world),
        ("Capsules", capsules3::init_world),
        ("CCD", ccd3::init_world),
        ("Collision Groups", collision_groups3::init_world),
        ("Compound Shapes", cross3::init_world),
        //        ("Compound Shapes", compound3::init_world::<Real),
        ("Constraints", constraints3::init_world),
        ("Convex Polyhedra", convex3::init_world),
        ("Conveyor Belt", conveyor_belt3::init_world),
        ("Damping", damping3::init_world),
        ("Dzhanibekov Effect", dzhanibekov3::init_world),
        ("FEM Volume", fem_volume3::init_world),
        ("Force Generator", force_generator3::init_world),
        ("Heightfield", heightfield3::init_world),
        ("Kinematic Body", kinematic3::init_world),
        //        ("Mass-constraint System", mass_constraint_system3::init_worl),
        //        ("Mass-spring System", mass_spring_system3::init_world),
        ("Multibody", multibody3::init_world),
        ("Plasticity", plasticity3::init_world),
        ("Ragdolls", ragdoll3::init_world),
        ("Sensor", sensor3::init_world),
        ("Triangle Mesh", trimesh3::init_world),
    ];

    builders.sort_by_key(|builder| builder.0);

    let i = builders
        .iter()
        .position(|builder| builder.0.to_camel_case().as_str() == demo.as_str())
        .unwrap_or(0);
    let testbed = Testbed::<Real>::from_builders(i, builders);

    testbed.run()
}
