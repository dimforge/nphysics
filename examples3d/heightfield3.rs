extern crate nalgebra as na;

use na::{Point3, Vector3, DMatrix};
use ncollide3d::shape::{Cuboid, ShapeHandle, HeightField, HeightFieldCellStatus};
use nphysics3d::object::{ColliderDesc, RigidBodyDesc, DefaultBodySet, DefaultColliderSet, Ground, BodyPartHandle};
use nphysics3d::force_generator::DefaultForceGeneratorSet;
use nphysics3d::joint::DefaultJointConstraintSet;
use nphysics3d::world::{DefaultDynamicWorld, DefaultColliderWorld};
use nphysics_testbed3d::Testbed;

use rand::{Rng, SeedableRng, rngs::StdRng};


pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let dynamic_world = DefaultDynamicWorld::new(Vector3::new(0.0, -9.81, 0.0));
    let collider_world = DefaultColliderWorld::new();
    let mut bodies = DefaultBodySet::new();
    let mut colliders = DefaultColliderSet::new();
    let joint_constraints = DefaultJointConstraintSet::new();
    let force_generators = DefaultForceGeneratorSet::new();

    /*
     * Setup a random ground.
     */
    let mut rng = StdRng::from_seed([0; 32]);
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

    let ground_handle = bodies.insert(Ground::new());
    let co = ColliderDesc::new(ShapeHandle::new(heightfield))
        .build(BodyPartHandle(ground_handle, 0));
    colliders.insert(co);

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

    for i in 0usize..num {
        for j in 0usize..num {
            for k in 0usize..num {
                let x = i as f32 * shift - centerx;
                let y = j as f32 * shift + centery + height;
                let z = k as f32 * shift - centerz;

                // Build the rigid body.
                let rb = RigidBodyDesc::new()
                    .translation(Vector3::new(x, y, z))
                    .build();
                let rb_handle = bodies.insert(rb);

                // Build the collider.
                let co = ColliderDesc::new(cuboid.clone())
                    .density(1.0)
                    .build(BodyPartHandle(rb_handle, 0));
                colliders.insert(co);
            }
        }
    }

    /*
     * Set up the testbed.
     */
    testbed.set_ground_handle(Some(ground_handle));
    testbed.set_world(dynamic_world, collider_world, bodies, colliders, joint_constraints, force_generators);
    testbed.look_at(Point3::new(-6.0, 6.0, -6.0), Point3::new(0.0, 1.0, 0.0));
}

fn main() {
    let testbed = Testbed::from_builders(0, vec![
        ("Heightfield", init_world),
    ]);

    testbed.run()
}
