extern crate nalgebra as na;
extern crate ncollide2d;
extern crate nphysics2d;
extern crate nphysics_testbed2d;

use na::{Isometry2, Point2, Unit, Vector2};
use ncollide2d::shape::{Cuboid, ShapeHandle};
use nphysics2d::joint::{CartesianJoint, PrismaticJoint, RevoluteJoint};
use nphysics2d::object::{BodyPartHandle, Material};
use nphysics2d::volumetric::Volumetric;
use nphysics2d::world::World;
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
    let ground_radx = 25.0;
    let ground_rady = 1.0;
    let ground_shape = ShapeHandle::new(Cuboid::new(Vector2::new(
        ground_radx - COLLIDER_MARGIN,
        ground_rady - COLLIDER_MARGIN,
    )));

    let ground_pos = Isometry2::new(-Vector2::y() * 5.0, na::zero());
    world.add_collider(
        COLLIDER_MARGIN,
        ground_shape,
        BodyPartHandle::ground(),
        ground_pos,
        Material::default(),
    );

    /*
     * Revolute joint.
     */
    let rad = 0.1;
    let num = 20;
    let revo = RevoluteJoint::new(0.0);
    let geom = ShapeHandle::new(Cuboid::new(Vector2::repeat(rad - COLLIDER_MARGIN)));
    let inertia = geom.inertia(1.0);
    let center_of_mass = geom.center_of_mass();
    let mut parent = BodyPartHandle::ground();

    for _ in 0usize..num {
        /*
         * Create the rigid body.
         */
        parent = world.add_multibody_link(
            parent,
            revo,
            na::zero(),
            Vector2::new(-rad * 3.0, 0.0),
            inertia,
            center_of_mass,
        );

        /*
         * Create the collider.
         */
        world.add_collider(
            COLLIDER_MARGIN,
            geom.clone(),
            parent,
            Isometry2::identity(),
            Material::default(),
        );
    }

    /*
     * Prismatic joint.
     */
    let ramp = Vector2::new(0.0, 1.0);
    let mut parent = BodyPartHandle::ground();

    let mut prism = PrismaticJoint::new(Unit::new_normalize(ramp), 0.0);
    prism.enable_max_offset(rad * 3.0);
    prism.enable_min_offset(-rad * 3.0);

    for _ in 0usize..num {
        parent = world.add_multibody_link(
            parent,
            prism,
            Vector2::new(-rad * 3.0, 0.0),
            na::zero(),
            inertia,
            center_of_mass,
        );
        world.add_collider(
            COLLIDER_MARGIN,
            geom.clone(),
            parent,
            Isometry2::identity(),
            Material::default(),
        );
    }

    /*
     * Rectangular joint.
     */
    let shift = Vector2::new(0.0, 2.0);
    let width = 5.0 * rad * 4.0;

    for i in 0..5 {
        for j in 0..5 {
            let mut x = i as f32 * rad * 4.0 - width / 2.0;
            let y = j as f32 * rad * 4.0 - width / 2.0;

            if j % 2 == 0 {
                x += rad * 2.0;
            }

            let mut rect = CartesianJoint::new(Vector2::new(x, y));
            let handle = world.add_multibody_link(
                BodyPartHandle::ground(),
                rect,
                shift,
                na::zero(),
                inertia,
                center_of_mass,
            );
            world.add_collider(
                COLLIDER_MARGIN,
                geom.clone(),
                handle,
                Isometry2::identity(),
                Material::default(),
            );
        }
    }

    /*
     * Set up the testbed.
     */
    let mut testbed = Testbed::new(world);
    testbed.look_at(Point2::new(1.0, 2.0), 130.0);
    testbed.run();
}
