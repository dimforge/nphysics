extern crate nalgebra as na;
extern crate ncollide2d;
extern crate nphysics2d;
extern crate nphysics_testbed2d;

use std::sync::Arc;
use std::f32;
use std::path::Path;
use na::{Isometry2, Point2, Point3, Vector2, Translation3};
use ncollide2d::shape::{Cuboid, ShapeHandle, Ball, Polyline};
use ncollide2d::procedural;
use ncollide2d::bounding_volume::{self, AABB, BoundingVolume};
use nphysics2d::object::{BodyPartHandle, Material, DeformableSurfaceDesc, ColliderDesc, RigidBodyDesc};
use nphysics2d::volumetric::Volumetric;
use nphysics2d::world::World;
use nphysics2d::math::Inertia;
use nphysics_testbed2d::Testbed;


const COLLIDER_MARGIN: f32 = 0.01;

fn main() {
    /*
     * World
     */
    let mut world = World::new();
    world.set_gravity(Vector2::new(0.0, -9.81));

    /*
     * Ground.
     */
    let obstacle = ShapeHandle::new(Cuboid::new(Vector2::repeat(0.2)));

    let mut obstacle_desc = ColliderDesc::new(obstacle);

    let _ = obstacle_desc
        .set_translation(Vector2::x() * 4.0)
        .build(&mut world);

    let _ = obstacle_desc
        .set_translation(Vector2::x() * -4.0)
        .build(&mut world);

    /*
     * Create the deformable body and a collider for its boundary.
     */
    let deformable_handle = DeformableSurfaceDesc::quad(50, 1)
        .with_scale(10.0, 1.0)
        .with_translation(Vector2::y() * 1.0)
        .with_young_modulus(1.0e4)
        .with_mass_damping(0.2)
        .with_boundary_polyline_collider(true)
        .build(&mut world)
        .handle();

    /*
     * Create a pyramid on top of the deformable body.
     */

    /*
     * Create the boxes
     */
    let num = 20;
    let rad = 0.1;
    let shift = 2.0 * rad;
    let centerx = shift * (num as f32) / 2.0;

    let cuboid = ShapeHandle::new(Cuboid::new(Vector2::repeat(rad)));
    let mut collider_desc = ColliderDesc::new(cuboid)
        .with_density(Some(1.0));

    let mut rb_desc = RigidBodyDesc::default()
        .with_collider(&collider_desc);

    for i in 0usize..num {
        for j in i..num {
            let fj = j as f32;
            let fi = i as f32;
            let x = (fi * shift / 2.0) + (fj - fi) * 2.0 * (rad + ColliderDesc::<f32>::default_margin()) - centerx;
            let y = fi * 2.0 * (rad + collider_desc.margin()) + rad + 4.0;

            // Build the rigid body and its collider.
            let _ = rb_desc
                .set_translation(Vector2::new(x, y))
                .build(&mut world);
        }
    }

    /*
     * Set up the testbed.
     */
    let mut testbed = Testbed::new(world);
    testbed.set_body_color(deformable_handle, Point3::new(0.0, 0.0, 1.0));
    testbed.look_at(Point2::new(0.0, -3.0), 100.0);
    testbed.run();
}
