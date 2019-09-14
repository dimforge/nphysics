#[macro_use]
extern crate kiss3d;
extern crate nalgebra as na;
#[cfg(feature = "dim2")]
extern crate ncollide2d as ncollide;
#[cfg(feature = "dim3")]
extern crate ncollide3d as ncollide;
#[cfg(feature = "dim2")]
extern crate nphysics2d as nphysics;
#[cfg(feature = "dim3")]
extern crate nphysics3d as nphysics;
extern crate num_traits as num;
#[macro_use]
extern crate bitflags;

#[cfg(feature = "log")]
#[macro_use]
extern crate log;

pub use crate::engine::GraphicsManager;
pub use crate::testbed::Testbed;

#[cfg(feature = "box2d-backend")]
mod box2d_world;
mod engine;
pub mod objects;
mod testbed;
mod ui;
