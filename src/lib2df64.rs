#[crate_id = "nphysics2df64#0.1"];
#[crate_type = "lib"];
#[warn(non_camel_case_types)];
#[feature(managed_boxes)];
#[feature(globs)];

extern mod std;
extern mod extra;
extern mod nalgebra;
extern mod ncollide = "ncollide2df64";

pub mod aliases;

pub mod integration;
pub mod detection;
pub mod resolution;

pub mod world;

pub mod object;

pub mod utils {
    pub mod ref_to;
}
