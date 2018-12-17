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
use nphysics2d::object::{BodyPartHandle, Material, DeformableSurface};
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
    let ground_size = 50.0;
    let ground_shape =
        ShapeHandle::new(Cuboid::new(Vector2::repeat(ground_size - COLLIDER_MARGIN)));
    let ground_pos = Isometry2::new(Vector2::y() * -ground_size, na::zero());

    world.add_collider(
        COLLIDER_MARGIN,
        ground_shape.clone(),
        BodyPartHandle::ground(),
        ground_pos,
        Material::default(),
    );


    let ground_size = 0.2;
    let ground_shape =
        ShapeHandle::new(Cuboid::new(Vector2::repeat(ground_size - COLLIDER_MARGIN)));
    let ground_pos = Isometry2::new(Vector2::new(0.0, 2.0), na::zero());
    world.add_collider(
        COLLIDER_MARGIN,
        ground_shape.clone(),
        BodyPartHandle::ground(),
        ground_pos,
        Material::default(),
    );

    let ground_pos = Isometry2::new(Vector2::new(7.0, 2.0), na::zero());
    world.add_collider(
        COLLIDER_MARGIN,
        ground_shape.clone(),
        BodyPartHandle::ground(),
        ground_pos,
        Material::default(),
    );


    /*
     * Create the deformable body and a collider for its contour.
     */
    let mut surface = DeformableSurface::quad(
        &Isometry2::new(Vector2::new(5.0, 6.0), 0.0),
        &Vector2::new(10.0, 1.0),
        50, 1,
        1.0, 1.0e3, 0.0,
        (0.0, 0.0));
    let (polyline, ids_map, parts_map) = surface.boundary_polyline();
    surface.renumber_dofs(&ids_map);

    let mut imin_x = 0;
    let mut imax_x = 0;

    for (i, p) in polyline.points().iter().enumerate() {
        let min_x = polyline.points()[imin_x].x;
        let min_y = polyline.points()[imin_x].y;
        let max_x = polyline.points()[imax_x].x;
        let max_y = polyline.points()[imax_x].y;

        if p.x < min_x || (p.x == min_x && p.y > min_y) {
            imin_x = i
        }

        if p.x > max_x || (p.x == max_x && p.y > max_y) {
            imax_x = i
        }
    }
    surface.set_node_kinematic(imin_x, true);
    surface.set_node_kinematic(imax_x, true);

    let deformable_handle = world.add_body(Box::new(surface));
    world.add_deformable_collider(
        COLLIDER_MARGIN,
        polyline,
        deformable_handle,
        None,
        Some(Arc::new(parts_map)),
        Material::default(),
    );
    world.body_mut(deformable_handle).set_deactivation_threshold(None);

    // Add a cube to play around with.
    let geom = ShapeHandle::new(Cuboid::new(Vector2::repeat(0.5 - COLLIDER_MARGIN)));
    let inertia = geom.inertia(0.1);
    let center_of_mass = geom.center_of_mass();
    let pos = Isometry2::new(Vector2::y() * 15.0, na::zero());
    let handle = world.add_rigid_body(pos, inertia, center_of_mass);

    world.add_collider(
        COLLIDER_MARGIN,
        geom.clone(),
        handle,
        Isometry2::identity(),
        Material::default(),
    );


    /*
     * Set up the testbed.
     */
    let mut testbed = Testbed::new(world);
    testbed.set_body_color(deformable_handle, Point3::new(0.0, 0.0, 1.0));
    // testbed.hide_performance_counters();
//    testbed.look_at(Point3::new(0.0, 0.0, 2.0), Point3::new(0.0, 0.0, 0.0));
    testbed.run();
}
