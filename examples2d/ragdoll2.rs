extern crate nalgebra as na;
extern crate ncollide2d;
extern crate nphysics2d;
extern crate nphysics_testbed2d;

use na::{Isometry2, Vector2};
use ncollide2d::shape::{Ball, Cuboid, ShapeHandle};
use nphysics2d::world::World;
use nphysics2d::object::{BodyHandle, Material};
use nphysics2d::joint::{RevoluteJoint, FreeJoint};
use nphysics2d::volumetric::Volumetric;
use nphysics_testbed2d::Testbed;

const COLLIDER_MARGIN: f32 = 0.01;

fn main() {
    /*
     * World
     */
    let mut world = World::new();
    world.set_gravity(Vector2::new(0.0, 9.81));

    /*
     * A plane for the ground
     */
    let ground_radx = 25.0;
    let ground_rady = 1.0;
    let ground_shape = ShapeHandle::new(Cuboid::new(Vector2::new(
        ground_radx - COLLIDER_MARGIN,
        ground_rady - COLLIDER_MARGIN,
    )));

    let ground_pos = Isometry2::new(Vector2::y() * ground_rady, na::zero());
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
    let n     = 5;
    let shiftx = 2.0;
    let shifty = 6.5;

    for i in 0usize .. n {
        for j in 0usize .. n {
            let x = i as f32 * shiftx - n as f32 * shiftx / 2.0;
            let y = j as f32 * (-shifty) - 6.0;

            add_ragdoll(Vector2::new(x, y), &mut world);
        }
    }

    /*
     * Run the simulation.
     */
    let mut testbed = Testbed::new(world);

    testbed.run();
}

fn add_ragdoll(pos: Vector2<f32>, world: &mut World<f32>) {
    let body_rady = 1.2;
    let body_radx = 0.2;
    let head_rad = 0.4;
    let member_rad = 0.1;
    let arm_length = 0.9;
    let leg_length = 1.4;
    let space = 0.1;

    let body_geom = ShapeHandle::new(Cuboid::new(Vector2::new(body_radx, body_rady)));
    let head_geom = ShapeHandle::new(Ball::new(head_rad));
    let arm_geom = ShapeHandle::new(Cuboid::new(Vector2::new(
        member_rad,
        arm_length,
    )));
    let leg_geom = ShapeHandle::new(Cuboid::new(Vector2::new(
        member_rad,
        leg_length,
    )));

    let body_inertia = body_geom.inertia(0.3);
    let head_inertia = head_geom.inertia(0.3);
    let arm_inertia = arm_geom.inertia(0.3);
    let leg_inertia = leg_geom.inertia(0.3);

    let body_center_of_mass = body_geom.center_of_mass();
    let head_center_of_mass = head_geom.center_of_mass();
    let arm_center_of_mass = arm_geom.center_of_mass();
    let leg_center_of_mass = leg_geom.center_of_mass();

    let free = FreeJoint::new(Isometry2::new(pos, na::zero()));
    let spherical = RevoluteJoint::new(na::zero());

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
        Isometry2::identity(),
        Material::default(),
    );

    /*
     * Head.
     */
    let head = world.add_multibody_link(
        body,
        spherical,
        Vector2::new(0.0, -(body_rady + head_rad + space * 2.0)),
        na::zero(),
        head_inertia,
        head_center_of_mass,
    );

    let _ = world.add_collider(
        COLLIDER_MARGIN,
        head_geom,
        head,
        Isometry2::identity(),
        Material::default(),
    );

    /*
     * Arms.
     */
    let left_arm = world.add_multibody_link(
        body,
        spherical,
        Vector2::new(body_radx + 2.0 * space, -body_rady),
        Vector2::new(0.0, -(arm_length + space)),
        arm_inertia,
        arm_center_of_mass,
    );

    let right_arm = world.add_multibody_link(
        body,
        spherical,
        Vector2::new(-body_radx - 2.0 * space, -body_rady),
        Vector2::new(0.0, -(arm_length + space)),
        arm_inertia,
        arm_center_of_mass,
    );

    let _ = world.add_collider(
        COLLIDER_MARGIN,
        arm_geom.clone(),
        left_arm,
        Isometry2::identity(),
        Material::default(),
    );

    let _ = world.add_collider(
        COLLIDER_MARGIN,
        arm_geom,
        right_arm,
        Isometry2::identity(),
        Material::default(),
    );

    /*
     * Legs.
     */
    let left_leg = world.add_multibody_link(
        body,
        spherical,
        Vector2::new(body_radx, body_rady),
        Vector2::new(0.0, -(leg_length + space)),
        leg_inertia,
        leg_center_of_mass,
    );

    let rigth_leg = world.add_multibody_link(
        body,
        spherical,
        Vector2::new(-body_radx, body_rady),
        Vector2::new(0.0, -(leg_length + space)),
        leg_inertia,
        leg_center_of_mass,
    );

    let _ = world.add_collider(
        COLLIDER_MARGIN,
        leg_geom.clone(),
        left_leg,
        Isometry2::identity(),
        Material::default(),
    );

    let _ = world.add_collider(
        COLLIDER_MARGIN,
        leg_geom,
        rigth_leg,
        Isometry2::identity(),
        Material::default(),
    );
}
