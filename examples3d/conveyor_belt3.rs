extern crate nalgebra as na;
extern crate ncollide3d;
extern crate nphysics3d;
extern crate nphysics_testbed3d;

use std::f32;
use na::{Point3, Vector3};
use ncollide3d::shape::{Cuboid, ShapeHandle};
use nphysics3d::object::{ColliderDesc, RigidBodyDesc};
use nphysics3d::material::{BasicMaterial, MaterialHandle};
use nphysics3d::world::World;
use nphysics_testbed3d::Testbed;

fn main() {
    /*
     * World
     */
    let mut world = World::new();
    world.set_gravity(Vector3::new(0.0, -9.81, 0.0));

    /*
     * Conveyor belts. We create 8 belts to form a "circle".
     */
    let conveyor_length = 5.0;
    let conveyor_half_width = 1.0;
    let conveyor_side_shape =
        ShapeHandle::new(Cuboid::new(Vector3::new(conveyor_length - conveyor_half_width, 0.2, conveyor_half_width)));
    let conveyor_corner_shape =
        ShapeHandle::new(Cuboid::new(Vector3::new(conveyor_half_width, 0.2, conveyor_half_width)));
    let conveyor_shift = conveyor_length;

    let conveyor_side_material = BasicMaterial { surface_velocity: Some(Vector3::new(1.0, 0.0, 0.0)), ..BasicMaterial::default() };
    let conveyor_corner_material = BasicMaterial { surface_velocity: Some(Vector3::new(1.0, 0.0, -2.0)), ..BasicMaterial::default() };

    // Build the sides of the conveyor belt "circle".
    //
    // Note that we can re-use the same material everywhere because the surface_velocity
    // is expressed in the local-space of the shape (and we are applying rotations to the
    // colliders so the surface_velocity points toward where we want it to).
    let mut side_desc = ColliderDesc::new(conveyor_side_shape.clone())
        .material(MaterialHandle::new(conveyor_side_material));

    side_desc
        .set_translation(Vector3::new(0.0, -0.2, conveyor_shift))
        .build(&mut world);

    side_desc
        .set_translation(Vector3::new(conveyor_shift, -0.2, 0.0))
        .set_rotation(Vector3::y() * f32::consts::FRAC_PI_2)
        .build(&mut world);

    side_desc
        .set_translation(Vector3::new(0.0, -0.2, -conveyor_shift))
        .set_rotation(Vector3::y() * f32::consts::PI)
        .build(&mut world);

    side_desc
        .set_translation(Vector3::new(-conveyor_shift, -0.2, 0.0))
        .set_rotation(Vector3::y() * (f32::consts::PI + f32::consts::FRAC_PI_2))
        .build(&mut world);

    // Build the corners of the conveyor belt "circle".

    // Note that we can re-use the same material everywhere because the surface_velocity
    // is expressed in the local-space of the shape (and we are applying rotations to the
    // colliders so the surface_velocity points toward where we want it to).
    let mut corner_desc = ColliderDesc::new(conveyor_corner_shape)
        .material(MaterialHandle::new(conveyor_corner_material));

    corner_desc
        .set_translation(Vector3::new(conveyor_shift, -0.2, conveyor_shift))
        .build(&mut world);

    corner_desc
        .set_translation(Vector3::new(conveyor_shift, -0.2, -conveyor_shift))
        .set_rotation(Vector3::y() * (f32::consts::FRAC_PI_2))
        .build(&mut world);

    corner_desc
        .set_translation(Vector3::new(-conveyor_shift, -0.2, -conveyor_shift))
        .set_rotation(Vector3::y() * f32::consts::PI)
        .build(&mut world);

    corner_desc
        .set_translation(Vector3::new(-conveyor_shift, -0.2, conveyor_shift))
        .set_rotation(Vector3::y() * (f32::consts::PI + f32::consts::FRAC_PI_2))
        .build(&mut world);

    /*
     * Create some boxes
     */
    let num = 4;
    let rad = 0.1;

    let ball = ShapeHandle::new(Cuboid::new(Vector3::repeat(rad)));
    let collider_desc = ColliderDesc::new(ball)
        .density(1.0);

    let mut rb_desc = RigidBodyDesc::new()
        .collider(&collider_desc);

    let shift = (rad + collider_desc.get_margin()) * 2.0;
    let centerx = shift * (num as f32) / 2.0 + conveyor_shift;
    let centery = shift / 2.0;
    let centerz = shift * (num as f32) / 2.0;
    let height = 0.0;

    for i in 0usize..num {
        for j in 0usize..num {
            for k in 0usize..num {
                let x = i as f32 * shift - centerx;
                let y = j as f32 * shift + centery + height;
                let z = k as f32 * shift - centerz;

                // Build the rigid body and its collider.
                rb_desc
                    .set_translation(Vector3::new(x, y, z))
                    .build(&mut world);
            }
        }
    }

    /*
     * Set up the testbed.
     */
    let mut testbed = Testbed::new(world);
    testbed.look_at(Point3::new(-10.0, 4.0, -10.0), Point3::new(0.0, 1.0, 0.0));
    testbed.run();
}
