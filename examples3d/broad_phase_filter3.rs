extern crate nalgebra as na;

use na::{Isometry3, Point3, RealField, Vector3};
use ncollide3d::broad_phase::BroadPhasePairFilter;
use ncollide3d::shape::{Ball, Capsule, Cuboid, ShapeHandle};
use nphysics3d::force_generator::DefaultForceGeneratorSet;
use nphysics3d::joint::{BallJoint, DefaultJointConstraintSet, FreeJoint};
use nphysics3d::object::{
    BodyPartHandle, BodySet, Collider, ColliderAnchor, ColliderDesc, ColliderSet,
    DefaultBodyHandle, DefaultBodySet, DefaultColliderHandle, DefaultColliderSet, Ground,
    MultibodyDesc,
};
use nphysics3d::world::{BroadPhasePairFilterSets, DefaultGeometricalWorld, DefaultMechanicalWorld};

use nphysics_testbed3d::Testbed;

struct NoMultibodySelfContactFilter;

impl<N, Bodies, Colliders> BroadPhasePairFilter<N, BroadPhasePairFilterSets<'_, N, Bodies, Colliders>>
    for NoMultibodySelfContactFilter
where
    N: RealField,
    Bodies: BodySet<N>,
    Colliders: ColliderSet<N, Bodies::Handle>,
{
    fn is_pair_valid(
        &self,
        h1: Colliders::Handle,
        h2: Colliders::Handle,
        set: &BroadPhasePairFilterSets<'_, N, Bodies, Colliders>,
    ) -> bool {
        let a1 = set.colliders().get(h1).map(|c| c.anchor());
        let a2 = set.colliders().get(h2).map(|c| c.anchor());

        match (a1, a2) {
            (
                Some(ColliderAnchor::OnBodyPart {
                    body_part: part1, ..
                }),
                Some(ColliderAnchor::OnBodyPart {
                    body_part: part2, ..
                }),
            ) => part1.0 != part2.0, // Don't collide if the two parts belong to the same body.
            _ => true,
        }
    }
}

/*
 * NOTE: The `r` macro is only here to convert from f64 to the `N` scalar type.
 * This simplifies experimentation with various scalar types (f32, fixed-point numbers, etc.)
 */
pub fn init_world<N: RealField>(testbed: &mut Testbed<N>) {
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
     * Ground.
     */
    let ground_thickness = r!(0.2);
    let ground_shape = ShapeHandle::new(Cuboid::new(Vector3::new(
        r!(5.0),
        ground_thickness,
        r!(5.0),
    )));

    let ground_handle = bodies.insert(Ground::new());
    let co = ColliderDesc::new(ground_shape)
        .translation(Vector3::y() * -ground_thickness)
        .build(BodyPartHandle(ground_handle, 0));
    colliders.insert(co);

    /*
     * Create the ragdolls
     */
    build_ragdolls(&mut bodies, &mut colliders);

    /*
     * Set up the testbed.
     */
    testbed.set_ground_handle(Some(ground_handle));
    testbed.set_broad_phase_pair_filter(NoMultibodySelfContactFilter);
    testbed.set_world(
        mechanical_world,
        geometrical_world,
        bodies,
        colliders,
        joint_constraints,
        force_generators,
    );
    testbed.look_at(Point3::new(-10.0, 5.0, -10.0), Point3::new(0.0, 1.0, 0.0));
}

fn build_ragdolls<N: RealField>(
    bodies: &mut DefaultBodySet<N>,
    colliders: &mut DefaultColliderSet<N>,
) {
    let body_rady = r!(1.2) / r!(2.0);
    let body_radz = r!(0.4) / r!(2.0);
    let body_radx = r!(0.2) / r!(2.0);
    let head_rad = r!(0.4) / r!(2.0);
    let member_rad = r!(0.15) / r!(2.0);
    let arm_length = r!(0.9) / r!(2.0);
    let leg_length = r!(1.4) / r!(2.0);
    let space = r!(0.15);

    let body_geom = ShapeHandle::new(Cuboid::new(Vector3::new(body_radx, body_rady, body_radz)));
    let head_geom = ShapeHandle::new(Ball::new(head_rad));
    let arm_geom = ShapeHandle::new(Capsule::new(arm_length, member_rad));
    let leg_geom = ShapeHandle::new(Capsule::new(leg_length, member_rad));

    // The position of the free joint will be modified in the
    // final loop of this function (before we actually build the
    // ragdoll body into the World.
    let free = FreeJoint::new(Isometry3::identity());
    let spherical = BallJoint::new(na::zero());

    /*
     * Body.
     */
    let body_collider = ColliderDesc::new(body_geom).density(r!(0.3));
    let mut body = MultibodyDesc::new(free);

    /*
     * Head.
     */
    let head_collider = ColliderDesc::new(head_geom).density(r!(0.3));
    body.add_child(spherical).set_parent_shift(Vector3::new(
        r!(0.0),
        body_rady + head_rad + space,
        r!(0.0),
    ));

    /*
     * Arms.
     */
    let arm_collider = ColliderDesc::new(arm_geom).density(r!(0.3));
    body.add_child(spherical)
        .set_parent_shift(Vector3::new(
            r!(0.0),
            body_rady,
            body_radx + r!(2.0) * space,
        ))
        .set_body_shift(Vector3::new(r!(0.0), arm_length + space, r!(0.0)));

    body.add_child(spherical)
        .set_parent_shift(Vector3::new(
            r!(0.0),
            body_rady,
            -body_radx - r!(2.0) * space,
        ))
        .set_body_shift(Vector3::new(r!(0.0), arm_length + space, r!(0.0)));

    /*
     * Legs.
     */
    let leg_collider = ColliderDesc::new(leg_geom).density(r!(0.3));
    body.add_child(spherical)
        .set_parent_shift(Vector3::new(r!(0.0), -body_rady, body_radx))
        .set_body_shift(Vector3::new(r!(0.0), leg_length + space, r!(0.0)));

    body.add_child(spherical)
        .set_parent_shift(Vector3::new(r!(0.0), -body_rady, -body_radx))
        .set_body_shift(Vector3::new(r!(0.0), leg_length + space, r!(0.0)));

    let n = 3;
    let shift = r!(1.0);
    let shifty = r!(5.0);

    for i in 0usize..n {
        for j in 0usize..n {
            for k in 0usize..n {
                let x = r!(i as f64) * shift - r!(n as f64) * shift / r!(2.0);
                let y = r!(j as f64) * shifty + r!(3.0);
                let z = r!(k as f64) * shift - r!(n as f64) * shift / r!(2.0);

                let free = FreeJoint::new(Isometry3::translation(x, y, z));
                let ragdoll = body.set_joint(free).build();
                let ragdoll_handle = bodies.insert(ragdoll);

                colliders.insert(body_collider.build(BodyPartHandle(ragdoll_handle, 0)));
                colliders.insert(head_collider.build(BodyPartHandle(ragdoll_handle, 1)));
                colliders.insert(arm_collider.build(BodyPartHandle(ragdoll_handle, 2)));
                colliders.insert(arm_collider.build(BodyPartHandle(ragdoll_handle, 3)));
                colliders.insert(leg_collider.build(BodyPartHandle(ragdoll_handle, 4)));
                colliders.insert(leg_collider.build(BodyPartHandle(ragdoll_handle, 5)));
            }
        }
    }
}

fn main() {
    let testbed = Testbed::<f32>::from_builders(0, vec![("Broad-phase filter", init_world)]);
    testbed.run()
}
