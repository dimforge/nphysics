#![warn(non_camel_case_types)]

extern crate nalgebra as na;
extern crate ncollide;
extern crate nphysics2d;
extern crate num_traits as num;
extern crate rand;
extern crate sfml;
extern crate time;

pub use testbed::Testbed;
pub use engine::{GraphicsManager, GraphicsManagerHandle};

mod testbed;
mod engine;

mod camera;
mod fps;
mod draw_helper;

pub mod objects;
