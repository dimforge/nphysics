extern crate nalgebra as na;
extern crate ncollide3d;
extern crate nphysics3d;
extern crate nphysics_testbed3d;

use na::{Isometry3, Point3, Vector3};
use ncollide3d::shape::{Ball, Cuboid, ShapeHandle};
use nphysics3d::world::World;
use nphysics3d::object::{BodyHandle, Material};
use nphysics3d::joint::{BallJoint, FreeJoint};
use nphysics3d::volumetric::Volumetric;
use nphysics_testbed3d::Testbed;

const COLLIDER_MARGIN: f32 = 0.01;

fn main() {
    /*
     * World
     */
    let mut world = World::new();
    world.set_gravity(Vector3::new(0.0, -9.81, 0.0));

    /*
     * Ground.
     */
    let ground_size = 50.0;
    let ground_shape =
        ShapeHandle::new(Cuboid::new(Vector3::repeat(ground_size - COLLIDER_MARGIN)));
    let ground_pos = Isometry3::new(Vector3::y() * -ground_size, na::zero());

    world.add_collider(
        COLLIDER_MARGIN,
        ground_shape,
        BodyHandle::ground(),
        ground_pos,
        Material::default(),
    );

    /*
     * Create the ragdolls
     */
    let n = 4;
    let shift = 5.0;
    let shifty = 6.0;

    for i in 0usize..n {
        for j in 0usize..n {
            for k in 0usize..n {
                let x = i as f32 * shift - n as f32 * shift / 2.0;
                let y = j as f32 * shifty + 10.0;
                let z = k as f32 * shift - n as f32 * shift / 2.0;

                add_ragdoll(Vector3::new(x, y, z), &mut world);
            }
        }
    }

    /*
     * Set up the testbed.
     */
    let mut testbed = Testbed::new(world);

    testbed.look_at(Point3::new(-30.0, 30.0, -30.0), Point3::new(0.0, 0.0, 0.0));
    testbed.run();
}

fn add_ragdoll(pos: Vector3<f32>, world: &mut World<f32>) {
    let body_rady = 1.2;
    let body_radz = 0.4;
    let body_radx = 0.2;
    let head_rad = 0.4;
    let member_rad = 0.1;
    let arm_length = 0.9;
    let leg_length = 1.4;
    let space = 0.1;

    let body_geom = ShapeHandle::new(Cuboid::new(Vector3::new(body_radx, body_rady, body_radz)));
    let head_geom = ShapeHandle::new(Ball::new(head_rad));
    let arm_geom = ShapeHandle::new(Cuboid::new(Vector3::new(
        member_rad,
        arm_length,
        member_rad,
    )));
    let leg_geom = ShapeHandle::new(Cuboid::new(Vector3::new(
        member_rad,
        leg_length,
        member_rad,
    )));

    let body_inertia = body_geom.inertia(0.3);
    let head_inertia = head_geom.inertia(0.3);
    let arm_inertia = arm_geom.inertia(0.3);
    let leg_inertia = leg_geom.inertia(0.3);

    let body_center_of_mass = body_geom.center_of_mass();
    let head_center_of_mass = head_geom.center_of_mass();
    let arm_center_of_mass = arm_geom.center_of_mass();
    let leg_center_of_mass = leg_geom.center_of_mass();

    let free = FreeJoint::new(Isometry3::new(pos, na::zero()));
    let spherical = BallJoint::new(na::zero());

    /*
     * Body.
     */
    let body = world.add_multibody_link(
        BodyHandle::ground(),
        free,
        na::zero(),
        na::zero(),
        body_inertia,
        body_center_of_mass,
    );

    let _ = world.add_collider(
        COLLIDER_MARGIN,
        body_geom,
        body,
        Isometry3::identity(),
        Material::default(),
    );

    /*
     * Head.
     */
    let head = world.add_multibody_link(
        body,
        spherical,
        Vector3::new(0.0, body_rady + head_rad + space * 2.0, 0.0),
        na::zero(),
        head_inertia,
        head_center_of_mass,
    );

    let _ = world.add_collider(
        COLLIDER_MARGIN,
        head_geom,
        head,
        Isometry3::identity(),
        Material::default(),
    );

    /*
     * Arms.
     */
    let left_arm = world.add_multibody_link(
        body,
        spherical,
        Vector3::new(0.0, body_rady, body_radz + 2.0 * space),
        Vector3::new(0.0, arm_length + space, 0.0),
        arm_inertia,
        arm_center_of_mass,
    );

    let right_arm = world.add_multibody_link(
        body,
        spherical,
        Vector3::new(0.0, body_rady, -body_radz - 2.0 * space),
        Vector3::new(0.0, arm_length + space, 0.0),
        arm_inertia,
        arm_center_of_mass,
    );

    let _ = world.add_collider(
        COLLIDER_MARGIN,
        arm_geom.clone(),
        left_arm,
        Isometry3::identity(),
        Material::default(),
    );

    let _ = world.add_collider(
        COLLIDER_MARGIN,
        arm_geom,
        right_arm,
        Isometry3::identity(),
        Material::default(),
    );

    /*
     * Legs.
     */
    let left_leg = world.add_multibody_link(
        body,
        spherical,
        Vector3::new(0.0, -body_rady, body_radz),
        Vector3::new(0.0, leg_length + space, 0.0),
        leg_inertia,
        leg_center_of_mass,
    );

    let rigth_leg = world.add_multibody_link(
        body,
        spherical,
        Vector3::new(0.0, -body_rady, -body_radz),
        Vector3::new(0.0, leg_length + space, 0.0),
        leg_inertia,
        leg_center_of_mass,
    );

    let _ = world.add_collider(
        COLLIDER_MARGIN,
        leg_geom.clone(),
        left_leg,
        Isometry3::identity(),
        Material::default(),
    );

    let _ = world.add_collider(
        COLLIDER_MARGIN,
        leg_geom,
        rigth_leg,
        Isometry3::identity(),
        Material::default(),
    );
}
