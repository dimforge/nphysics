#[crate_id = "graphics3d"];
#[crate_type = "lib"];
#[warn(non_camel_case_types)];
#[feature(managed_boxes)];

extern crate std;
extern crate extra;
extern crate glfw = "glfw-rs";
extern crate kiss3d;
extern crate nphysics = "nphysics3df32";
extern crate nalgebra;
extern crate ncollide = "ncollide3df32";


pub mod simulate;
pub mod engine;

// mod draw_helper;

pub mod objects {
    pub mod ball;
    pub mod box_node;
    pub mod plane;
    pub mod cylinder;
    pub mod cone;
    pub mod mesh;
}
