extern crate nalgebra as na;
extern crate ncollide2d;
extern crate nphysics2d;
extern crate nphysics_testbed2d;

use std::f32;
use na::{Point2, Point3, Vector2};
use ncollide2d::shape::{Cuboid, ShapeHandle};
use nphysics2d::object::{MassConstraintSystemDesc, ColliderDesc, RigidBodyDesc};
use nphysics2d::world::World;
use nphysics_testbed2d::Testbed;


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

    obstacle_desc
        .set_translation(Vector2::x() * 4.0)
        .build(&mut world);

    obstacle_desc
        .set_translation(Vector2::x() * -4.0)
        .build(&mut world);

    /*
     * Create the deformable body and a collider for its boundary.
     */
    let deformable_handle = MassConstraintSystemDesc::quad(50, 1)
        .scale(Vector2::new(10.0, 1.0))
        .translation(Vector2::y() * 1.0)
        .stiffness(Some(1.0e4))
        .collider_enabled(true)
        .build(&mut world);

    let mut deformable = world.mass_constraint_system_mut(deformable_handle).unwrap();

    // Add other constraints for volume stiffness.
    deformable.generate_neighbor_constraints(Some(1.0e4));
    deformable.generate_neighbor_constraints(Some(1.0e4));

    let nnodes = deformable.num_nodes();
    let extra_constraints1 = (0..).map(|i| Point2::new(i, nnodes - i - 2)).take(nnodes / 2);
    let extra_constraints2 = (1..).map(|i| Point2::new(i, nnodes - i)).take(nnodes / 2);

    for constraint in extra_constraints1.chain(extra_constraints2) {
        deformable.add_constraint(constraint.x, constraint.y, Some(1.0e4));
    }

    drop(deformable);

    /*
     * Create a pyramid on top of the deformable body.
     */
    let num = 20;
    let rad = 0.1;
    let shift = 2.0 * rad;
    let centerx = shift * (num as f32) / 2.0;

    let cuboid = ShapeHandle::new(Cuboid::new(Vector2::repeat(rad)));
    let collider_desc = ColliderDesc::new(cuboid)
        .density(0.1);

    let mut rb_desc = RigidBodyDesc::new()
        .collider(&collider_desc);

    for i in 0usize..num {
        for j in i..num {
            let fj = j as f32;
            let fi = i as f32;
            let x = (fi * shift / 2.0) + (fj - fi) * 2.0 * (rad + ColliderDesc::<f32>::default_margin()) - centerx;
            let y = fi * 2.0 * (rad + collider_desc.get_margin()) + rad + 2.0;

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
    testbed.set_body_color(deformable_handle, Point3::new(0.0, 0.0, 1.0));
    testbed.look_at(Point2::new(0.0, -3.0), 100.0);
    testbed.run();
}
