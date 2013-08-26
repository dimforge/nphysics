#[link(name     = "graphics2d"
       , vers   = "0.0"
       , author = "Sébastien Crozet"
       , uuid   = "313bf372-135c-46e1-8012-392061a1196")];
#[crate_type = "lib"];
#[warn(non_camel_case_types)]

extern mod std;
extern mod extra;
extern mod rsfml;
extern mod nphysics;
extern mod nalgebra;
extern mod ncollide;


pub mod simulate;
pub mod engine;

mod camera;
mod fps;
pub mod draw_helper;

mod objects {
    mod ball;
    mod box;
}
