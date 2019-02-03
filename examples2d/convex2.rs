extern crate nalgebra as na;
extern crate ncollide2d;
extern crate nphysics2d;
extern crate nphysics_testbed2d;
extern crate rand;

use rand::distributions::{Standard, Distribution};
use rand::{SeedableRng, XorShiftRng};

use na::{Point2, Vector2};
use ncollide2d::shape::{ConvexPolygon, Cuboid, ShapeHandle};
use nphysics2d::object::{ColliderDesc, RigidBodyDesc};
use nphysics2d::world::World;
use nphysics_testbed2d::Testbed;


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
        ShapeHandle::new(Cuboid::new(Vector2::new(ground_size, 1.0)));

    ColliderDesc::new(ground_shape)
        .translation(-Vector2::y())
        .build(&mut world);

    /*
     * Create the convex geometries.
     */
    let npts = 10usize;
    let num = 25;
    let shift = 0.4;
    let centerx = shift * (num as f32) / 2.0;
    let centery = shift;
    let mut rng = XorShiftRng::seed_from_u64(0);
    let distribution = Standard;

    for i in 0usize..num {
        for j in 0usize..num {
            let x = i as f32 * shift - centerx;
            let y = j as f32 * shift + centery;

            let mut pts = Vec::with_capacity(npts);

            for _ in 0..npts {
                let pt: Point2<f32> = distribution.sample(&mut rng);
                pts.push(pt * 0.4);
            }

            let geom = ShapeHandle::new(ConvexPolygon::try_from_points(&pts).unwrap());
            let collider_desc = ColliderDesc::new(geom)
                .density(1.0);

            RigidBodyDesc::new()
                .collider(&collider_desc)
                .translation(Vector2::new(x, y))
                .build(&mut world);
        }
    }

    /*
     * Set up the testbed.
     */
    let mut testbed = Testbed::new(world);
    testbed.look_at(Point2::new(0.0, -2.5), 95.0);
    testbed.run();
}
