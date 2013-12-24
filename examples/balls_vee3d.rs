#[crate_type = "bin"];
#[warn(non_camel_case_types)];
#[feature(managed_boxes)];

extern mod std;
extern mod extra;
extern mod kiss3d;
extern mod graphics3d;
extern mod nphysics;
extern mod nalgebra;
extern mod ncollide;

use kiss3d::window::Window;
use nalgebra::na::{Vec3, Translation};
use ncollide::geom::{Ball, Plane};
use nphysics::world::BodyWorld;
use nphysics::aliases::dim3;
use nphysics::object::{RigidBody, Static, Dynamic, RB};
use graphics3d::engine::GraphicsManager;


#[start]
fn start(argc: int, argv: **u8) -> int {
    std::rt::start_on_main_thread(argc, argv, main)
}

fn main() {
    GraphicsManager::simulate(balls_vee_3d)
}

pub fn balls_vee_3d(window: &mut Window, graphics: &mut GraphicsManager) -> dim3::BodyWorld3d<f32> {
    /*
     * World
     */
    let mut world = BodyWorld::new();
    world.set_gravity(Vec3::new(0.0f32, -9.81, 0.0));

    /*
     * Planes
     */
    let normals = [
        Vec3::new(-1.0f32, 1.0, -1.0 ),
        Vec3::new(1.0f32, 1.0, -1.0 ),
        Vec3::new(-1.0f32, 1.0, 1.0 ),
        Vec3::new(1.0f32, 1.0, 1.0 )
    ];
    for n in normals.iter() {
        let rb   = RigidBody::new(Plane::new(*n), 0.0f32, Static, 0.3, 0.6);
        let body = @mut RB(rb);

        world.add_body(body);
        graphics.add(window, body);
    }

    /*
     * Create the balls
     */
    let num     = (1500.0f32.pow(&(1.0f32 / 3.0))) as uint;
    let rad     = 0.5;
    let shift   = 2.5 * rad;
    let centerx = shift * (num as f32) / 2.0;
    let centery = shift * (num as f32) / 2.0;

    for i in range(0u, num) {
        for j in range(0u, num) {
            for k in range(0u, num) {
                let x = i as f32 * 2.5 * rad - centerx;
                let y = 10.0 + j as f32 * 2.5 * rad + centery * 2.0;
                let z = k as f32 * 2.5 * rad - centerx;

                let mut rb = RigidBody::new(Ball::new(rad), 1.0f32, Dynamic, 0.3, 0.6);

                rb.append_translation(&Vec3::new(x, y, z));

                let body = @mut RB(rb);

                world.add_body(body);
                graphics.add(window, body);
            }
        }
    }

    /*
     * Set up the camera and that is it!
     */
    graphics.look_at(Vec3::new(-10.0, 50.0, -10.0), Vec3::new(0.0, 0.0, 0.0));

    world
}
