extern crate nalgebra as na;
extern crate ncollide3d;
extern crate nphysics3d;
extern crate nphysics_testbed3d;
extern crate rand;

use na::{Vector3, Point3, DMatrix};
use ncollide3d::shape::{Cuboid, ShapeHandle, HeightField, HeightFieldCellStatus};
use nphysics3d::object::{ColliderDesc, RigidBodyDesc};
use nphysics3d::world::World;
use nphysics_testbed3d::Testbed;
use rand::{Rng, SeedableRng, StdRng};


pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut world = World::new();
    world.set_gravity(Vector3::new(0.0, -9.81, 0.0));

    /*
     * Setup a random ground.
     */
    let mut rng: StdRng = SeedableRng::from_seed([0; 32]);
    let heights = DMatrix::from_fn(10, 15, |_, _| rng.gen::<f32>());

    let mut heightfield: HeightField<f32> = HeightField::new(heights, Vector3::new(10.0, 1.5, 10.0));
    let statuses = heightfield.cells_statuses_mut();

    // It is possible to customize the way the triangles
    // are generated for each cell of the heightfield.
    for (i, stat) in statuses.iter_mut().enumerate() {
        if i % 2 == 0 {
            stat.insert(HeightFieldCellStatus::ZIGZAG_SUBDIVISION);
        }
    }

    // It is possible to remove some triangles from the heightfield.
    statuses.column_mut(9).apply(|status| status | HeightFieldCellStatus::RIGHT_TRIANGLE_REMOVED);
    statuses.column_mut(10).apply(|status| status | HeightFieldCellStatus::LEFT_TRIANGLE_REMOVED);
    statuses.row_mut(5).apply(|status| status | HeightFieldCellStatus::CELL_REMOVED);

    ColliderDesc::new(ShapeHandle::new(heightfield))
        .build(&mut world);

    /*
     * Create some boxes and spheres.
     */
    let num = 7;
    let rad = 0.1;
    let shift = rad * 2.0 + 0.5;
    let centerx = shift * (num as f32) / 2.0;
    let centery = shift / 2.0;
    let centerz = shift * (num as f32) / 2.0;
    let height = 1.0;

    let cuboid = ShapeHandle::new(Cuboid::new(Vector3::repeat(rad)));
    let collider_desc = ColliderDesc::new(cuboid)
        .density(1.0);

    let mut rb_desc = RigidBodyDesc::new()
        .collider(&collider_desc);

    for i in 0usize..num {
        for j in 0usize..num {
            for k in 0usize..num {
                let x = i as f32 * shift - centerx;
                let y = j as f32 * shift + centery + height;
                let z = k as f32 * shift - centerz;

                // Build the rigid body and its collider.
                rb_desc
                    .set_translation(Vector3::new(x, y, z))
                    .build(&mut world);
            }
        }
    }

    /*
     * Set up the testbed.
     */
    testbed.set_world(world);
    testbed.look_at(Point3::new(-6.0, 6.0, -6.0), Point3::new(0.0, 1.0, 0.0));
}

fn main() {
    let mut testbed = Testbed::new_empty();
    init_world(&mut testbed);
    testbed.run();
}
