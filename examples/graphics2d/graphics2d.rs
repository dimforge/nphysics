#[crate_id = "graphics2d"];
#[crate_type = "lib"];
#[warn(non_camel_case_types)];
#[feature(managed_boxes)];

extern mod std;
extern mod extra;
extern mod rsfml;
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
    pub mod box_node;
    pub mod lines;
}
