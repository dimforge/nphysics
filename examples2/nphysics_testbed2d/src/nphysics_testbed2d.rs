#![warn(non_camel_case_types)]

extern crate rand;
extern crate time;
extern crate collections;
extern crate rsfml;
extern crate "nalgebra" as na;
extern crate ncollide;
extern crate nphysics;


pub use testbed::Testbed;

mod testbed;
mod engine;

mod camera;
mod fps;
mod draw_helper;

mod objects {
    pub mod ball;
    pub mod box_node;
    pub mod lines;
    pub mod segment;
}
