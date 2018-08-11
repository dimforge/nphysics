extern crate kiss3d;
extern crate nalgebra as na;
extern crate ncollide2d;
extern crate nphysics2d;
extern crate num_traits as num;
extern crate rand;
#[macro_use]
extern crate log;
extern crate time;

pub use engine::GraphicsManager;
pub use testbed::Testbed;

mod engine;
pub mod objects;
mod testbed;
