extern crate nalgebra as na;

use na::{DMatrix, Point3, Vector3};
use ncollide3d::shape::{Cuboid, HeightField, HeightFieldCellStatus, ShapeHandle};
use nphysics3d::force_generator::DefaultForceGeneratorSet;
use nphysics3d::joint::DefaultJointConstraintSet;
use nphysics3d::object::{
    BodyPartHandle, ColliderDesc, DefaultBodySet, DefaultColliderSet, Ground, RigidBodyDesc,
};
use nphysics3d::world::{DefaultGeometricalWorld, DefaultMechanicalWorld};
use nphysics_testbed3d::{r, Real, Testbed};

use rand::{rngs::StdRng, Rng, SeedableRng};

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mechanical_world = DefaultMechanicalWorld::new(Vector3::new(r!(0.0), r!(-9.81), r!(0.0)));
    let geometrical_world = DefaultGeometricalWorld::new();
    let mut bodies = DefaultBodySet::new();
    let mut colliders = DefaultColliderSet::new();
    let joint_constraints = DefaultJointConstraintSet::new();
    let force_generators = DefaultForceGeneratorSet::new();

    /*
     * Setup a random ground.
     */
    let mut rng = StdRng::from_seed([0; 32]);
    let heights = DMatrix::from_fn(10, 15, |_, _| rng.gen::<Real>());

    let mut heightfield: HeightField<Real> =
        HeightField::new(heights, Vector3::new(r!(10.0), r!(1.5), r!(10.0)));
    let statuses = heightfield.cells_statuses_mut();

    // It is possible to customize the way the triangles
    // are generated for each cell of the heightfield.
    for (i, stat) in statuses.iter_mut().enumerate() {
        if i % 2 == 0 {
            stat.insert(HeightFieldCellStatus::ZIGZAG_SUBDIVISION);
        }
    }

    // It is possible to remove some triangles from the heightfield.
    statuses
        .column_mut(9)
        .apply(|status| status | HeightFieldCellStatus::RIGHT_TRIANGLE_REMOVED);
    statuses
        .column_mut(10)
        .apply(|status| status | HeightFieldCellStatus::LEFT_TRIANGLE_REMOVED);
    statuses
        .row_mut(5)
        .apply(|status| status | HeightFieldCellStatus::CELL_REMOVED);

    let ground_handle = bodies.insert(Ground::new());
    let co =
        ColliderDesc::new(ShapeHandle::new(heightfield)).build(BodyPartHandle(ground_handle, 0));
    colliders.insert(co);

    /*
     * Create some boxes and spheres.
     */
    let num = 7;
    let rad = r!(0.1);
    let shift = rad * r!(2.0) + 0.5;
    let centerx = shift * r!(num as f32) / r!(2.0);
    let centery = shift / r!(2.0);
    let centerz = shift * r!(num as f32) / r!(2.0);
    let height = r!(1.0);

    let cuboid = ShapeHandle::new(Cuboid::new(Vector3::repeat(rad)));

    for i in 0usize..num {
        for j in 0usize..num {
            for k in 0usize..num {
                let x = r!(i as f32) * shift - centerx;
                let y = r!(j as f32) * shift + centery + height;
                let z = r!(k as f32) * shift - centerz;

                // Build the rigid body.
                let rb = RigidBodyDesc::new()
                    .translation(Vector3::new(x, y, z))
                    .build();
                let rb_handle = bodies.insert(rb);

                // Build the collider.
                let co = ColliderDesc::new(cuboid.clone())
                    .density(r!(1.0))
                    .build(BodyPartHandle(rb_handle, 0));
                colliders.insert(co);
            }
        }
    }

    /*
     * Set up the testbed.
     */
    testbed.set_ground_handle(Some(ground_handle));
    testbed.set_world(
        mechanical_world,
        geometrical_world,
        bodies,
        colliders,
        joint_constraints,
        force_generators,
    );
    testbed.look_at(Point3::new(-6.0, 6.0, -6.0), Point3::new(0.0, 1.0, 0.0));
}

fn main() {
    let testbed = Testbed::from_builders(0, vec![("Heightfield", init_world)]);

    testbed.run()
}
