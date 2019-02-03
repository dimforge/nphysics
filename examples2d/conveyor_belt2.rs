extern crate nalgebra as na;
extern crate ncollide2d;
extern crate nphysics2d;
extern crate nphysics_testbed2d;

use na::{Point2, Vector2};
use ncollide2d::shape::{Cuboid, ShapeHandle};
use nphysics2d::object::{ColliderDesc, RigidBodyDesc};
use nphysics2d::material::{MaterialHandle, BasicMaterial};
use nphysics2d::world::World;
use nphysics_testbed2d::Testbed;


fn main() {
    /*
     * World
     */
    let mut world = World::new();
    world.set_gravity(Vector2::new(0.0, -9.81));

    /*
     * Ground
     */
    let ground_size = 5.0;
    let ground_shape = ShapeHandle::new(Cuboid::new(Vector2::new(ground_size, 0.2)));
    let conveyor_material1 = BasicMaterial { surface_velocity: Some(Vector2::x()), ..BasicMaterial::default() };
    let conveyor_material2 = BasicMaterial { surface_velocity: Some(-Vector2::x()), ..BasicMaterial::default() };

    for i in 0..10 {
        ColliderDesc::new(ground_shape.clone())
            .translation(Vector2::new(-2.0, 5.0 - i as f32 * 4.0))
            .rotation(0.1)
            .material(MaterialHandle::new(conveyor_material1))
            .build(&mut world);


        ColliderDesc::new(ground_shape.clone())
            .translation(Vector2::new(2.0, 3.0 - i as f32 * 4.0))
            .rotation(-0.1)
            .material(MaterialHandle::new(conveyor_material2))
            .build(&mut world);
    }

    /*
     * Create the boxes
     */
    let num = 5;
    let rad = 0.1;

    let cuboid = ShapeHandle::new(Cuboid::new(Vector2::repeat(rad)));
    let collider_desc = ColliderDesc::new(cuboid)
        .density(1.0);

    let mut rb_desc = RigidBodyDesc::new()
        .collider(&collider_desc);

    let shift = (rad + collider_desc.get_margin()) * 2.0;
    let centerx = shift * (num as f32) / 2.0 + 5.0;
    let centery = shift / 2.0 + 5.0;

    for i in 0usize..num {
        for j in 0..num {
            let x = i as f32 * shift - centerx;
            let y = j as f32 * shift + centery;

            // Build the rigid body and its collider.
            rb_desc
                .set_translation(Vector2::new(x, y))
                .build(&mut world);
        }
    }

    /*
     * Set up the testbed.
     */
    let mut testbed = Testbed::new(world);
    testbed.look_at(Point2::new(0.0, -2.5), 95.0);
    testbed.run();
}
