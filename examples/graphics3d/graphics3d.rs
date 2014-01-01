#[crate_id = "graphics3d"];
#[crate_type = "lib"];
#[warn(non_camel_case_types)];
#[feature(managed_boxes)];

extern mod std;
extern mod extra;
extern mod glfw;
extern mod kiss3d;
extern mod nphysics = "nphysics3df32";
extern mod nalgebra;
extern mod ncollide = "ncollide3df32";


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
