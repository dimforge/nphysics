#![warn(non_camel_case_types)]
#![feature(managed_boxes)]

extern crate rand;
extern crate time;
extern crate collections;
extern crate glfw;
extern crate kiss3d;
extern crate "nphysics3df32" as nphysics;
extern crate "nalgebra" as na;
extern crate ncollide;


pub use testbed::Testbed;

mod testbed;
mod engine;
mod objects;
