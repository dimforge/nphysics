#[link(name     = "graphics2d"
       , vers   = "0.0"
       , author = "Sébastien Crozet"
       , uuid   = "313bf372-135c-46e1-8012-392061a1196")];
#[crate_type = "lib"];
#[warn(non_camel_case_types)];
#[feature(managed_boxes)];

extern mod std;
extern mod extra;
extern mod rsfml = "rust-sfml";
extern mod nphysics;
extern mod nalgebra;
extern mod ncollide;


pub mod simulate;
pub mod engine;

pub mod camera;
pub mod fps;
pub mod draw_helper;

pub mod objects {
    pub mod ball;
    pub mod box;
}
