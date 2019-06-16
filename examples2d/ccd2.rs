extern crate nalgebra as na;
extern crate ncollide2d;
extern crate nphysics2d;
extern crate nphysics_testbed2d;

use na::{Point2, Vector2, Isometry2};
use ncollide2d::shape::{Ball, Cuboid, ShapeHandle};
use nphysics2d::object::{ColliderDesc, RigidBodyDesc};
use nphysics2d::world::World;
use nphysics2d::math::Velocity;
use nphysics2d::material::{BasicMaterial, MaterialHandle};
use nphysics_testbed2d::Testbed;


pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut world = World::new();
    world.set_gravity(Vector2::new(0.0, -9.81));

    /*
     * Ground
     */
    let ground_size = 25.0;
    let ground_shape =
        ShapeHandle::new(Cuboid::new(Vector2::new(ground_size, 0.1)));

    ColliderDesc::new(ground_shape.clone())
        .ccd_enabled(true)
        .build(&mut world);

    ColliderDesc::new(ground_shape.clone())
        .position(Isometry2::new(Vector2::new(-3.0, 0.0), 3.14 / 2.0))
        .ccd_enabled(true)
        .build(&mut world);

    ColliderDesc::new(ground_shape.clone())
        .position(Isometry2::new(Vector2::new(3.0, 0.0), 3.14 / 2.0))
        .ccd_enabled(true)
        .build(&mut world);

    ColliderDesc::new(ground_shape)
        .position(Isometry2::translation(0.0, 10.0))
        .ccd_enabled(true)
        .build(&mut world);

    /*
     * Create the balls
     */
    let num = 5;
    let rad = 0.1;

    let ball = ShapeHandle::new(Cuboid::new(Vector2::new(rad * 4.0, rad))); // Ball::new(rad));
    let collider_desc = ColliderDesc::new(ball)
//        .margin(1.0)
        .ccd_enabled(true)
//        .position(Isometry2::rotation(0.5))
//        .material(MaterialHandle::new(BasicMaterial::new(2.0, 0.0)))
        .density(1.0);

    let mut rb_desc = RigidBodyDesc::new()
        .collider(&collider_desc);

    let shift = (rad + collider_desc.get_margin()) * 2.0;
    let centerx = shift * (num as f32) / 2.0 + 1.0;
    let centery = shift / 2.0 + 6.0;

    for i in 0usize..num {
        for j in 0..num {
            let x = i as f32 * shift * 4.0 - centerx;
            let y = j as f32 * shift + centery;

            // Build the rigid body and its collider.
            rb_desc
                .set_translation(Vector2::new(x, y))
                .set_velocity(Velocity::linear(100.0, -100.0))
//                .set_rotation_kinematic(true)
                .build(&mut world);
        }
    }

    /*
     * Set up the testbed.
     */
    testbed.set_world(world);
    testbed.look_at(Point2::new(0.0, -2.5), 95.0);
}

fn main() {
    let mut testbed = Testbed::new_empty();
    init_world(&mut testbed);
    testbed.run();
}