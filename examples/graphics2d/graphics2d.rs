#[crate_id = "graphics2d"];
#[crate_type = "lib"];
#[warn(non_camel_case_types)];
#[feature(managed_boxes)];

extern crate std;
extern crate rand;
extern crate extra;
extern crate sync;
extern crate time;
extern crate collections;
extern crate rsfml;
extern crate nphysics = "nphysics2df32";
extern crate nalgebra;
extern crate ncollide = "ncollide2df32";


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
