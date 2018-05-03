extern crate nalgebra as na;
extern crate ncollide3d;
extern crate nphysics3d;
extern crate nphysics_testbed3d;
extern crate rand;

use rand::{Rng, SeedableRng, StdRng};
use na::{Isometry3, Point3, Vector3};
use ncollide3d::shape::{Cuboid, ShapeHandle, TriMesh};
use nphysics3d::volumetric::Volumetric;
use nphysics3d::world::World;
use nphysics3d::object::{BodyHandle, Material};
use nphysics_testbed3d::Testbed;

const COLLIDER_MARGIN: f32 = 0.01;

fn main() {
    /*
     * World
     */
    let mut world = World::new();
    world.set_gravity(Vector3::new(0.0, -9.81, 0.0));

    /*
     * A tesselated Bézier surface for the ground.
     */
    let quad = ncollide3d::procedural::quad(10.0, 10.0, 10, 10);
    let indices = quad.flat_indices()
        .chunks(3)
        .map(|is| Point3::new(is[0] as usize, is[1] as usize, is[2] as usize))
        .collect();
    let mut rng: StdRng = SeedableRng::from_seed(&[1, 2, 3, 4][..]);
    let mut vertices = quad.coords;

    // ncollide generatse a quad with `z` as the normal.
    // so we switch z and y here and set a random altitude at each point.
    for p in &mut vertices {
        p.z = p.y;
        p.y = rng.gen::<f32>() * 1.5;
    }

    let trimesh: TriMesh<f32> = TriMesh::new(vertices, indices, None);
    world.add_collider(
        COLLIDER_MARGIN,
        ShapeHandle::new(trimesh),
        BodyHandle::ground(),
        Isometry3::identity(),
        Material::default(),
    );

    /*
     * Create some boxes and spheres.
     */
    let num = 8;
    let rad = 0.1;
    let shift = rad * 2.0 + 0.5;
    let centerx = shift * (num as f32) / 2.0;
    let centery = shift / 2.0;
    let centerz = shift * (num as f32) / 2.0;
    let height = 2.0;

    let geom = ShapeHandle::new(Cuboid::new(Vector3::repeat(rad - COLLIDER_MARGIN)));
    let inertia = geom.inertia(1.0);
    let center_of_mass = geom.center_of_mass();

    for i in 0usize..num {
        for j in 0usize..num {
            for k in 0usize..num {
                let x = i as f32 * shift - centerx;
                let y = j as f32 * shift + centery + height;
                let z = k as f32 * shift - centerz;

                /*
                 * Create the rigid body.
                 */
                let pos = Isometry3::new(Vector3::new(x, y, z), na::zero());
                let handle = world.add_rigid_body(pos, inertia, center_of_mass);

                /*
                 * Create the collider.
                 */
                world.add_collider(
                    COLLIDER_MARGIN,
                    geom.clone(),
                    handle,
                    Isometry3::identity(),
                    Material::default(),
                );
            }
        }
    }

    /*
     * Set up the testbed.
     */
    let mut testbed = Testbed::new(world);

    testbed.run();
}
