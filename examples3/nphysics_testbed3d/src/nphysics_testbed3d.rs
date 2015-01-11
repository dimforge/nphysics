#![allow(unstable)]
#![warn(non_camel_case_types)]

extern crate rand;
extern crate time;
extern crate collections;
extern crate glfw;
extern crate kiss3d;
extern crate "nalgebra" as na;
extern crate ncollide;
extern crate nphysics;


pub use testbed::Testbed;

mod testbed;
mod engine;
mod objects;
