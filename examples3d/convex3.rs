extern crate nalgebra as na;
extern crate ncollide3d;
extern crate nphysics3d;
extern crate nphysics_testbed3d;
extern crate rand;

use rand::distributions::{Standard, Distribution};
use rand::{SeedableRng, XorShiftRng};

use na::{Point3, Vector3};
use ncollide3d::shape::{ConvexHull, Cuboid, ShapeHandle};
use nphysics3d::object::{ColliderDesc, RigidBodyDesc};
use nphysics3d::world::World;
use nphysics_testbed3d::Testbed;


fn main() {
    /*
     * World
     */
    let mut world = World::new();
    world.set_gravity(Vector3::new(0.0, -9.81, 0.0));

    /*
     * Ground
     */
    let ground_size = 50.0;
    let ground_shape =
        ShapeHandle::new(Cuboid::new(Vector3::repeat(ground_size)));

    ColliderDesc::new(ground_shape)
        .with_translation(Vector3::y() * -ground_size)
        .build(&mut world);

    /*
     * Create the convex geometries.
     */
    let npts = 10usize;
    let num = 6;
    let shift = 0.4;
    let centerx = shift * (num as f32) / 2.0;
    let centery = shift / 2.0;
    let centerz = shift * (num as f32) / 2.0;
    let mut rng = XorShiftRng::seed_from_u64(0);
    let distribution = Standard;

    for i in 0usize..num {
        for j in 0usize..num {
            for k in 0usize..num {
                let x = i as f32 * shift - centerx;
                let y = j as f32 * shift + centery;
                let z = k as f32 * shift - centerz;

                let mut pts = Vec::with_capacity(npts);

                for _ in 0..npts {
                    let pt: Point3<f32> = distribution.sample(&mut rng);
                    pts.push(pt * 0.4);
                }

                let geom = ShapeHandle::new(ConvexHull::try_from_points(&pts).unwrap());
                let collider_desc = ColliderDesc::new(geom)
                    .with_density(1.0);

                RigidBodyDesc::default()
                    .with_collider(&collider_desc)
                    .with_translation(Vector3::new(x, y, z))
                    .build(&mut world);
            }
        }
    }

    /*
     * Set up the testbed.
     */
    let mut testbed = Testbed::new(world);

    testbed.look_at(Point3::new(-4.0, 1.0, -4.0), Point3::new(0.0, 1.0, 0.0));
    testbed.run();
}
