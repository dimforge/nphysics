#[link(name     = "graphics3d"
       , vers   = "0.0"
       , author = "Sébastien Crozet"
       , uuid   = "204cc370-78ff-4717-9de3-e2ebaea74d85")];
#[crate_type = "lib"];
#[warn(non_camel_case_types)]

extern mod std;
extern mod extra;
extern mod kiss3d;
extern mod nphysics;
extern mod nalgebra;
extern mod ncollide;


pub mod simulate;
pub mod engine;

// mod draw_helper;

mod objects {
    mod ball;
    mod box;
    mod plane;
    mod cylinder;
    mod cone;
}
