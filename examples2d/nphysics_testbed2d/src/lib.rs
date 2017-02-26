#![warn(non_camel_case_types)]

extern crate rand;
extern crate time;
extern crate num_traits as num;
extern crate sfml;
extern crate nalgebra as na;
extern crate ncollide;
extern crate nphysics2d;


pub use testbed::Testbed;
pub use engine::{GraphicsManager, GraphicsManagerHandle};
pub use testbed::{CallBackMode, CallBackId};

mod testbed;
mod engine;

mod camera;
mod fps;
mod draw_helper;

pub mod objects;
