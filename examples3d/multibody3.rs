extern crate nalgebra as na;
extern crate ncollide3d;
extern crate nphysics3d;
extern crate nphysics_testbed3d;

use std::f32::consts::PI;
use std::sync::Arc;
use na::{Isometry3, Point3, Real, Translation3, Unit, Vector2, Vector3};
use ncollide3d::shape::{Ball, Cuboid, Plane, ShapeHandle};
use nphysics3d::world::World;
use nphysics3d::object::{BodyHandle, Material};
use nphysics3d::joint::{BallJoint, CartesianJoint, CylindricalJoint, FixedJoint, FreeJoint,
                        HelicalJoint, Joint, PinSlotJoint, PlanarJoint, PrismaticJoint,
                        RectangularJoint, RevoluteJoint, UniversalJoint};
use nphysics3d::volumetric::Volumetric;
use nphysics_testbed3d::Testbed;

const COLLIDER_MARGIN: f32 = 0.01;

fn main() {
    /*
     * World
     */
    let mut world = World::new();
    world.set_gravity(Vector3::new(0.0, -9.81, 0.0));
    // world.integration_parameters_mut().max_position_iterations = 0;
    world.integration_parameters_mut().warmstart_coeff = 0.0;
    world.integration_parameters_mut().max_velocity_iterations = 50;
    world.integration_parameters_mut().dt = 0.0001;

    /*
     * Create a ground.
     */
    // let ground_shape = ShapeHandle::new(Cuboid::new(Vector3::new(50.0, 50.0, 50.0)));
    // let ground_pos   = Isometry3::new(Vector3::y() * -57.0, na::zero());
    // world.add_collider(COLLIDER_MARGIN, ground_shape, BodyHandle::ground(), ground_pos, Material::default());

    /*
     * Revolute joints.
     */
    let num = 20;
    let rad = 0.2;
    let mut parent = BodyHandle::ground();

    // let revo    = RevoluteJoint::new(Vector3::x_axis(), 0.0);
    let revo = BallJoint::new(na::zero());
    let geom = ShapeHandle::new(Ball::new(rad));
    let inertia = geom.inertia(1.0);
    let center_of_mass = geom.center_of_mass();

    for _ in 0usize..num {
        let body_shift = Vector3::z() * rad * 3.0;
        parent = world.add_multibody_link(
            parent,
            revo,
            na::zero(),
            body_shift,
            inertia,
            center_of_mass,
        );
        world.add_collider(
            COLLIDER_MARGIN,
            geom.clone(),
            parent,
            Isometry3::identity(),
            Material::default(),
        );
    }

    // Setup damping for the whole multibody.
    // world.multibody_mut(parent).unwrap().damping_mut().fill(0.1);

    // /*
    //  * Prismatic joint.
    //  */
    // parent = BodyHandle::ground();
    // let mut prism = PrismaticJoint::new(Vector3::y_axis(), 0.0);
    // prism.enable_min_offset(-rad * 3.0);

    // for _ in 0usize .. num {
    //     let parent_shift = Vector3::z() * rad * 3.0;
    //     parent = world.add_multibody_link(parent, prism, parent_shift, na::zero(), inertia, center_of_mass);
    //     world.add_collider(COLLIDER_MARGIN, geom.clone(), parent, Isometry3::identity(), Material::default());
    // }

    /*
     * Ball joint.
     */
    // parent = BodyHandle::ground();
    // for i in 0usize .. num {
    //     // The multibody links are initialized along a circle.
    //     let angle = i as f32 * 2.0 * PI / (num as f32);
    //     let shift = rad * 3.0;
    //     let mut body_shift   = Vector3::new(angle.cos(), 0.0, angle.sin()) * shift;
    //     let mut parent_shift = Vector3::zeros();

    //     if i == 0 {
    //         parent_shift.y = -3.0 * rad;
    //     }

    //     let spherical = BallJoint::new(na::zero());
    //     parent = world.add_multibody_link(parent, spherical, parent_shift, body_shift, inertia, center_of_mass);
    //     world.add_collider(COLLIDER_MARGIN, geom.clone(), parent, Isometry3::identity(), Material::default());
    // }
    // // Setup damping for the whole multibody.
    // world.multibody_mut(parent).unwrap().damping_mut().fill(0.1);

    // /*
    //  * Planar joint.
    //  */
    // let cuboid = ShapeHandle::new(Cuboid::new(Vector3::repeat(rad)));
    // let cuboid_inertia = cuboid.inertia(1.0);
    // let cuboid_center_of_mass = cuboid.center_of_mass();
    // let axis1  = Unit::new_normalize(Vector3::new(1.0, 0.0, 1.0));
    // let axis2  = Vector3::y_axis();
    // let shift  = Vector3::y();
    // for i in 0 .. 5 {
    //     for j in 0 .. 5 {
    //         let x = i as f32 * rad * 4.0;
    //         let y = j as f32 * rad * 4.0;
    //         let planar = PlanarJoint::new(axis1, axis2, x, y, 0.4);
    //         let handle = world.add_multibody_link(BodyHandle::ground(), planar, shift, na::zero(), cuboid_inertia, cuboid_center_of_mass);
    //         world.add_collider(COLLIDER_MARGIN, cuboid.clone(), handle, Isometry3::identity(), Material::default());
    //     }
    // }

    // /*
    //  * Rectangular joint.
    //  */
    // let cuboid = ShapeHandle::new(Cuboid::new(Vector3::repeat(rad)));
    // let cuboid_inertia = cuboid.inertia(1.0);
    // let axis1  = Unit::new_normalize(Vector3::new(1.0, 0.0, 1.0));
    // let axis2  = Vector3::y_axis();
    // let shift  = Vector3::y();
    // for i in 0 .. 5 {
    //     for j in 0 .. 5 {
    //         let x = i as f32 * rad * 4.0;
    //         let y = j as f32 * rad * 4.0 + 5.0;
    //         let planar = RectangularJoint::new(axis1, axis2, x, y);
    //         let handle = world.add_multibody_link(BodyHandle::ground(), planar, shift, na::zero(), cuboid_inertia, cuboid_center_of_mass);
    //         world.add_collider(COLLIDER_MARGIN, cuboid.clone(), handle, Isometry3::identity(), Material::default());
    //     }
    // }

    // // /*
    // //  * Cartesian joint.
    // //  */
    // // let cuboid = ShapeHandle::new(Cuboid::new(Vector3::repeat(rad)));
    // // let cuboid_inertia = cuboid.inertia(1.0);
    // // for i in 0 .. 5 {
    // //     for j in 0 .. 5 {
    // //         let x = i as f32 * rad * 4.0;
    // //         let z = j as f32 * rad * 4.0 + 2.0;
    // //         let planar = CartesianJoint::new(Vector3::new(x, 10.0, z));
    // //         let handle = world.add_multibody_link(BodyHandle::ground(), planar,
    // //                                               na::zero(), na::zero(), cuboid_inertia);
    // //         world.add_collider(COLLIDER_MARGIN, cuboid.clone(), handle, Isometry3::identity(), Material::default());
    // //     }
    // // }

    // /*
    //  * Cylindrical joint.
    //  */
    // let cuboid         = ShapeHandle::new(Cuboid::new(Vector3::new(rad * 5.0, rad * 5.0, rad)));
    // let cuboid_inertia = cuboid.inertia(1.0);
    // let parent_shift   = Vector3::y() * -3.0;
    // let axis           = -Vector3::z_axis();

    // let cyl    = CylindricalJoint::new(axis, 1.0, 0.0);
    // let handle = world.add_multibody_link(BodyHandle::ground(), cyl, parent_shift, na::zero(), cuboid_inertia, cuboid_center_of_mass);
    // world.add_collider(COLLIDER_MARGIN, cuboid.clone(), handle, Isometry3::identity(), Material::default());

    // /*
    //  * Pin-slot joint.
    //  */
    // let cuboid         = ShapeHandle::new(Cuboid::new(Vector3::new(rad * 5.0, rad, rad * 5.0)));
    // let cuboid_inertia = cuboid.inertia(1.0);
    // let axis_v         = Unit::new_normalize(Vector3::new(0.0, 1.0, 1.0));
    // let axis_w         = Vector3::x_axis();

    // let pin_slot = PinSlotJoint::new(axis_v, axis_w, 5.0, 0.0);
    // let handle   = world.add_multibody_link(BodyHandle::ground(), pin_slot, na::zero(), na::zero(), cuboid_inertia, cuboid_center_of_mass);
    // world.add_collider(COLLIDER_MARGIN, cuboid.clone(), handle, Isometry3::identity(), Material::default());

    // /*
    //  * Helical joint.
    //  */
    // let cuboid = ShapeHandle::new(Cuboid::new(Vector3::repeat(rad)));
    // let cuboid_inertia = cuboid.inertia(1.0);
    // let axis = Vector3::y_axis();

    // let hel = HelicalJoint::new(axis, 1.0, 10.0);
    // let handle = world.add_multibody_link(BodyHandle::ground(), hel, na::zero(), na::zero(), cuboid_inertia, cuboid_center_of_mass);
    // world.add_collider(COLLIDER_MARGIN, cuboid.clone(), handle, Isometry3::identity(), Material::default());

    // /*
    //  * Universal joint.
    //  */
    // let axis1 = Vector3::x_axis();
    // let axis2 = Vector3::z_axis();
    // let uni = UniversalJoint::new(axis1, axis2, 0.0, 0.0);
    // let handle = world.add_multibody_link(BodyHandle::ground(), uni, Vector3::new(0.0, 5.0, -3.0), -Vector3::z() * rad * 3.0, cuboid_inertia, cuboid_center_of_mass);
    // world.add_collider(COLLIDER_MARGIN, cuboid.clone(), handle, Isometry3::identity(), Material::default());

    // world.multibody_link_mut(handle).unwrap().joint_velocity_mut()[1] = 10.0;

    /*
     * Set up the testbed.
     */
    let mut testbed = Testbed::new(world);
    testbed.set_number_of_steps_per_frame(160);
    testbed.look_at(Point3::new(30.0, -2.0, 0.0), Point3::new(0.0, -2.0, 0.0));
    testbed.run();
}
