#![allow(dead_code)]

extern crate nalgebra as na;

macro_rules! r(
    ($e: expr) => {
        nalgebra::convert::<f64, N>($e)
    }
);

use inflector::Inflector;

use nphysics_testbed2d::Testbed;

mod balls2;
mod boxes2;
mod broad_phase_filter2;
mod capsules2;
mod ccd2;
mod ccd_trigger2;
mod collision_groups2;
mod compound2;
mod constraints2;
mod convex2;
mod conveyor_belt2;
mod cross2;
mod damping2;
mod fem_surface2;
mod force_generator2;
mod heightfield2;
mod kinematic2;
mod mass_constraint_system2;
mod mass_spring_system2;
mod multibody2;
mod plasticity2;
mod polyline2;
mod ragdoll2;
mod sensor2;

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
        ("Balls", balls2::init_world),
        ("Boxes", boxes2::init_world),
        ("Broad-phase filter", broad_phase_filter2::init_world),
        ("Capsules", capsules2::init_world),
        ("CCD", ccd2::init_world),
        ("CCD Trigger", ccd_trigger2::init_world),
        ("Collision Groups", collision_groups2::init_world),
        ("Compound Shapes", cross2::init_world),
        //        ("Compound Shapes", compound2::init_world),
        ("Constraints", constraints2::init_world),
        ("Convex Polygons", convex2::init_world),
        ("Conveyor Belt", conveyor_belt2::init_world),
        ("Damping", damping2::init_world),
        ("FEM Surface", fem_surface2::init_world),
        ("Force Generator", force_generator2::init_world),
        ("Heightfield", heightfield2::init_world),
        ("Kinematic body", kinematic2::init_world),
        (
            "Mass-constraint System",
            mass_constraint_system2::init_world,
        ),
        ("Mass-spring System", mass_spring_system2::init_world),
        ("Multibody", multibody2::init_world),
        ("Plasticity", plasticity2::init_world),
        ("Ragdolls", ragdoll2::init_world),
        ("Sensor", sensor2::init_world),
        ("Polygonal Line", polyline2::init_world),
    ];

    builders.sort_by_key(|builder| builder.0);

    let i = builders
        .iter()
        .position(|builder| builder.0.to_camel_case().as_str() == demo.as_str())
        .unwrap_or(0);
    let testbed = Testbed::<Real>::from_builders(i, builders);

    testbed.run()
}
