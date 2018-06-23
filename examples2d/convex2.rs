extern crate nalgebra as na;
extern crate ncollide2d;
extern crate nphysics2d;
extern crate nphysics_testbed2d;
extern crate rand;

use rand::{Rand, XorShiftRng};

use na::{Isometry2, Point2, Vector2};
use ncollide2d::shape::{Ball, ConvexPolygon, Cuboid, ShapeHandle};
use nphysics2d::object::{BodyHandle, Material};
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
     * Ground
     */
    let ground_size = 50.0;
    let ground_shape =
        ShapeHandle::new(Cuboid::new(Vector2::repeat(ground_size - COLLIDER_MARGIN)));
    let ground_pos = Isometry2::new(Vector2::y() * -ground_size, na::zero());

    world.add_collider(
        COLLIDER_MARGIN,
        ground_shape,
        BodyHandle::ground(),
        ground_pos,
        Material::default(),
    );

    /*
     * Create the convex geometries.
     */
    let npts = 10usize;
    let num = 25;
    let shift = 0.4;
    let centerx = shift * (num as f32) / 2.0;
    let centery = shift;
    let mut rng = XorShiftRng::new_unseeded();

    for i in 0usize..num {
        for j in 0usize..num {
            let x = i as f32 * shift - centerx;
            let y = j as f32 * shift + centery;

            let mut pts = Vec::with_capacity(npts);

            for _ in 0..npts {
                pts.push(Point2::rand(&mut rng) * 0.4);
            }

            let geom = ShapeHandle::new(ConvexPolygon::try_from_points(&pts).unwrap());

            let inertia = geom.inertia(1.0);
            let center_of_mass = geom.center_of_mass();

            let pos = Isometry2::new(Vector2::new(x, y), na::zero());
            let handle = world.add_rigid_body(pos, inertia, center_of_mass);

            /*
             * Create the collider.
             */
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
    testbed.look_at(Point2::new(0.0, -2.5), 95.0);
    testbed.run();
}
