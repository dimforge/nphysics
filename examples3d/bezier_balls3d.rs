extern crate native;
extern crate "nalgebra" as na;
extern crate ncollide;
extern crate "nphysics3df32" as nphysics;
extern crate nphysics_testbed3d;

use na::{Vec3, Translation};
use ncollide::geom::{Ball, BezierSurface};
use nphysics::world::World;
use nphysics::object::RigidBody;
use nphysics_testbed3d::Testbed;

#[start]
fn start(argc: int, argv: *const *const u8) -> int {
    native::start(argc, argv, main)
}

fn main() {
    /*
     * World
     */
    let mut world = World::new();
    world.set_gravity(Vec3::new(0.0f32, -9.81, 0.0));

    /*
     * Ground
     */
    let control_points = vec!(
        Vec3::new(-20.0, -00.0, 020.0), Vec3::new(-10.0, -00.0, 020.0), Vec3::new(00.0, -00.0, 020.0), Vec3::new(10.0, -00.0, 020.0), Vec3::new(20.0, -00.0, 020.0),
        Vec3::new(-20.0, -00.0, 010.0), Vec3::new(-10.0, -40.0, 010.0), Vec3::new(00.0, 040.0, 010.0), Vec3::new(10.0, -40.0, 010.0), Vec3::new(20.0, -00.0, 010.0),
        Vec3::new(-20.0, -00.0, 000.0), Vec3::new(-10.0, -40.0, 000.0), Vec3::new(00.0, 040.0, 000.0), Vec3::new(10.0, -40.0, 000.0), Vec3::new(20.0, -00.0, 000.0),
        Vec3::new(-20.0, -00.0, -10.0), Vec3::new(-10.0, -40.0, -10.0), Vec3::new(00.0, 040.0, -10.0), Vec3::new(10.0, -40.0, -10.0), Vec3::new(20.0, -00.0, -10.0),
        Vec3::new(-20.0, -00.0, -20.0), Vec3::new(-10.0, -00.0, -20.0), Vec3::new(00.0, -00.0, -20.0), Vec3::new(10.0, -00.0, -20.0), Vec3::new(20.0, -00.0, -20.0),
    );

    let rb = RigidBody::new_static(BezierSurface::new(control_points, 5, 5), 0.3, 0.6);

    world.add_body(rb);

    /*
     * Create the balls
     */
    let num     = 150.0f32.powf(1.0f32 / 3.0) as uint;
    let rad     = 0.5;
    let shift   = 2.5 * rad;
    let centerx = shift * (num as f32) / 2.0;
    let centery = shift * (num as f32) / 2.0;

    for i in range(0u, num) {
        for j in range(0u, num) {
            for k in range(0u, num) {
                let x = i as f32 * 2.5 * rad - centerx;
                let y = j as f32 * 2.5 * rad + centery * 2.0;
                let z = k as f32 * 2.5 * rad - centerx;

                let mut rb = RigidBody::new_dynamic(Ball::new(rad), 1.0f32, 0.3, 0.6);

                rb.append_translation(&Vec3::new(x, y, z));

                world.add_body(rb);
            }
        }
    }

    /*
     * Set up the testbed.
     */
    let mut testbed = Testbed::new(world);

    testbed.look_at(Vec3::new(-10.0, 50.0, -10.0), Vec3::new(0.0, 0.0, 0.0));
    testbed.run();
}
