extern crate nalgebra as na;
extern crate ncollide2d;
extern crate nphysics2d;
extern crate nphysics_testbed2d;

use na::{Isometry2, Vector2};
use ncollide2d::shape::{Cuboid, ShapeHandle};
use nphysics2d::object::Material;
use nphysics2d::volumetric::Volumetric;
use nphysics2d::world::World;
use nphysics_testbed2d::Testbed;

const COLLIDER_MARGIN: f32 = 0.01;

fn main() {
    /*
     * World
     */
    let mut world = World::new();
    world.set_gravity(Vector2::new(0.0, 0.0));

    /*
     * Create the box that will be deactivated.
     */
    let rad = 0.1;
    let geom = ShapeHandle::new(Cuboid::new(Vector2::new(rad, rad)));
    let inertia = geom.inertia(1.0);
    let center_of_mass = geom.center_of_mass();

    /*
     * Create the body that will be deactivated.
     */
    let body1 = world.add_rigid_body(Isometry2::identity(), inertia, center_of_mass);
    world
        .rigid_body_mut(body1)
        .unwrap()
        .set_linear_velocity(Vector2::new(0.099, 0.0));

    /*
     * Create the body that cannot be deactivated.
     */
    let pos2 = Isometry2::new(Vector2::y() * 0.3, na::zero());
    let body2 = world.add_rigid_body(pos2, inertia, center_of_mass);

    {
        let rb = world.rigid_body_mut(body2).unwrap();
        rb.set_linear_velocity(Vector2::new(0.1, 0.0));
        rb.activation_status_mut().set_deactivation_threshold(None);
    }

    // (Optional:) add colliders just so the testbed displays something.
    world.add_collider(
        COLLIDER_MARGIN,
        geom.clone(),
        body1,
        Isometry2::identity(),
        Material::default(),
    );

    world.add_collider(
        COLLIDER_MARGIN,
        geom.clone(),
        body2,
        Isometry2::identity(),
        Material::default(),
    );

    /*
     * Set up the testbed.
     */
    let testbed = Testbed::new(world);
    testbed.run();
}
