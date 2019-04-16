extern crate nalgebra as na;
extern crate ncollide3d;
extern crate nphysics3d;
extern crate nphysics_testbed3d;

use na::{Point3, Vector3, Point4};
use ncollide3d::shape::{Cuboid, ShapeHandle};
use nphysics3d::object::{FEMVolumeDesc, ColliderDesc};
use nphysics3d::world::World;
use nphysics_testbed3d::Testbed;

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut world = World::new();
    world.set_gravity(Vector3::new(0.0, -9.81, 0.0));

    /*
     * Ground.
     */
    let ground_thickness = 0.2;
    let ground = ShapeHandle::new(Cuboid::new(Vector3::new(3.0, ground_thickness, 3.0)));

    ColliderDesc::new(ground)
        .translation(Vector3::y() * (-ground_thickness - 1.0))
        .build(&mut world);


    let ground_size = 3.0;
    let obstacle = ShapeHandle::new(Cuboid::new(Vector3::new(0.02, 0.02, ground_size)));

    let mut obstacle_desc = ColliderDesc::new(obstacle);

    obstacle_desc
        .set_translation(Vector3::new(0.4, -0.01, 0.0))
        .build(&mut world);

    obstacle_desc
        .set_translation(Vector3::new(-0.4, -0.01, 0.0))
        .build(&mut world);

    /*
     * Create the deformable body and a collider for its boundary.
     */
    FEMVolumeDesc::cube(20, 1, 1)
        .scale(Vector3::new(1.0, 0.1, 0.1))
        .translation(Vector3::y() * 0.1)
        .young_modulus(1.0e3)
        .poisson_ratio(0.2)
        .mass_damping(0.2)
        .collider_enabled(true)
        .build(&mut world);

    /*
     * Set up the testbed.
     */
    testbed.set_world(world);
    testbed.look_at(Point3::new(0.0, 0.0, 2.0), Point3::new(0.0, 0.0, 0.0));
}

fn main() {
    let mut testbed = Testbed::new_empty();
    init_world(&mut testbed);
    testbed.run();
}
