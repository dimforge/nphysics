#[crate_id = "nphysics#0.1"];
#[crate_type = "lib"];
#[warn(non_camel_case_types)];
#[feature(managed_boxes)];
#[feature(globs)];

extern mod std;
extern mod extra;
extern mod nalgebra;
extern mod ncollide;

pub mod aliases {
    pub mod dim2;
    pub mod dim3;
    pub mod traits;
}

pub mod signal {
    pub mod signal;
}

pub mod integration;
pub mod detection;
pub mod resolution;

pub mod world;

pub mod object;
