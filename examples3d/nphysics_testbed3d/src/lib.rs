extern crate rand;
extern crate time;
extern crate num_traits as num;
extern crate glfw;
extern crate kiss3d;
extern crate nalgebra as na;
extern crate ncollide;
extern crate nphysics3d;


pub use testbed::Testbed;
pub use engine::{GraphicsManager, GraphicsManagerHandle};

mod testbed;
mod engine;
pub mod objects;
