#[link(name     = "compound2d"
       , vers   = "0.0"
       , author = "Sébastien Crozet"
       , uuid   = "f31fbf6a-b82c-4a8a-857a-12b2387f20ae")];
#[crate_type = "bin"];
#[warn(non_camel_case_types)];
#[feature(managed_boxes)];

extern mod std;
extern mod extra;
extern mod rsfml;
extern mod nphysics;
extern mod nalgebra;
extern mod ncollide;
extern mod graphics2d;

use extra::arc::Arc;
use nalgebra::na::{Vec2, Iso2, Translation};
use nalgebra::na;
use ncollide::geom::{Geom, CompoundAABB};
use nphysics::world::BodyWorld;
use nphysics::aliases::dim2;
use nphysics::object::{RigidBody, Static, Dynamic, RB};
use graphics2d::engine::GraphicsManager;

#[start]
fn start(argc: int, argv: **u8) -> int {
    std::rt::start_on_main_thread(argc, argv, main)
}

fn main() {
    GraphicsManager::simulate(compound_2d)
}

pub fn compound_2d(graphics: &mut GraphicsManager) -> dim2::BodyWorld2d<f32> {
    /*
     * World
     */
    let mut world = BodyWorld::new();
    world.set_gravity(Vec2::new(0.0f32, 9.81));

    /*
     * First plane
     */
    let mut rb = RigidBody::new(Geom::new_plane(Vec2::new(-1.0f32, -1.0)), 0.0f32, Static, 0.3, 0.6);

    rb.append_translation(&Vec2::new(0.0, 10.0));

    let body = @mut RB(rb);

    world.add_body(body);
    graphics.add(body);

    /*
     * Second plane
     */
    let mut rb = RigidBody::new(Geom::new_plane(Vec2::new(1.0f32, -1.0)), 0.0f32, Static, 0.3, 0.6);

    rb.append_translation(&Vec2::new(0.0, 10.0));

    let body = @mut RB(rb);

    world.add_body(body);
    graphics.add(body);

    /*
     * Cross shaped geometry
     */
    let delta1 = Iso2::new(Vec2::new(0.0f32, -5.0), na::zero());
    let delta2 = Iso2::new(Vec2::new(-5.0f32, 0.0), na::zero());
    let delta3 = Iso2::new(Vec2::new(5.0f32,  0.0), na::zero());

    let mut cross_geoms = ~[];
    cross_geoms.push((delta1, Geom::new_box(Vec2::new(5.0f32, 0.75))));
    cross_geoms.push((delta2, Geom::new_box(Vec2::new(0.75f32, 5.0))));
    cross_geoms.push((delta3, Geom::new_box(Vec2::new(0.75f32, 5.0))));

    let cross = Arc::new(CompoundAABB::new(cross_geoms));

    /*
     * Create the boxes
     */
    let num     = (750.0f32.sqrt()) as uint;
    let rad     = 5.0;
    let shift   = 2.5 * rad;
    let centerx = shift * (num as f32) / 2.0;
    let centery = shift * (num as f32) / 2.0;

    for i in range(0u, num) {
        for j in range(0u, num) {
            let x = i as f32 * 2.5 * rad - centerx;
            let y = j as f32 * 2.5 * rad - centery * 2.0 - 250.0;

            let mut rb = RigidBody::new(Geom::new_compound(cross.clone()), 1.0f32, Dynamic, 0.3, 0.6);

            rb.append_translation(&Vec2::new(x, y));

            let body = @mut RB(rb);

            world.add_body(body);
            graphics.add(body);
        }
    }

    /*
     * The end.
     */
    world
}
