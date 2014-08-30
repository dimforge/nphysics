#![warn(non_camel_case_types)]
#![feature(managed_boxes)]

extern crate rand;
extern crate sync;
extern crate time;
extern crate collections;
extern crate rsfml;
extern crate nphysics = "nphysics2df32";
extern crate nalgebra;
extern crate ncollide = "ncollide2df32";


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
}
