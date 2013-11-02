#[link(name         = "nphysics"
       , package_id = "nphysics"
       , vers       = "0.0"
       , author     = "Sébastien Crozet"
       , uuid       = "b55cde38-0873-4861-bcb6-041fcb58d1bd")];
#[crate_type = "lib"];
#[warn(non_camel_case_types)];
#[feature(managed_boxes)];

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
