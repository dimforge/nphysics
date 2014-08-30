#![warn(non_camel_case_types)]
#![feature(managed_boxes)]

extern crate rand;
extern crate time;
extern crate collections;
extern crate glfw;
extern crate kiss3d;
extern crate nphysics = "nphysics3df32";
extern crate nalgebra;
extern crate ncollide = "ncollide3df32";


pub use testbed::Testbed;

mod testbed;
mod engine;
mod objects;
