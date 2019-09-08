use na::RealField;

use crate::counters::Counters;
use crate::solver::{IntegrationParameters, MoreauJeanSolver};
use crate::material::MaterialsCoefficientsTable;
use crate::object::{BodySet, ColliderSet, BodyStatus, Body};
use crate::joint::{JointConstraintSet, JointConstraint};
use crate::detection::ColliderContactManifold;
use crate::world::GeometricalWorld;


// XXX: Find a better name.
pub trait MechanicalSolver<
    N: RealField,
    Bodies: BodySet<N>,
    Colliders: ColliderSet<N, Bodies::Handle>,
    Constraints: JointConstraintSet<N, Bodies>
> {
    fn solve(
        &mut self,
        gworld: &mut GeometricalWorld<N, Bodies::Handle, Colliders::Handle>,
        counters: &mut Counters,
        bodies: &mut Bodies,
        colliders: &mut Colliders,
        constraints: &mut Constraints,
        parameters: &IntegrationParameters<N>,
        material_coefficients: &MaterialsCoefficientsTable<N>,
        active_bodies: &[Bodies::Handle], // FIXME: we should create a dedicated structure for islands.
    );
}

impl<N, Bodies, Colliders, Constraints> MechanicalSolver<N, Bodies, Colliders, Constraints> for MoreauJeanSolver<N, Bodies, Colliders::Handle>
    where N: RealField,
          Bodies: BodySet<N>,
          Colliders: ColliderSet<N, Bodies::Handle>,
          Constraints: JointConstraintSet<N, Bodies> {
    fn solve(
        &mut self,
        gworld: &mut GeometricalWorld<N, Bodies::Handle, Colliders::Handle>,
        counters: &mut Counters,
        bodies: &mut Bodies,
        colliders: &mut Colliders,
        constraints: &mut Constraints,
        parameters: &IntegrationParameters<N>,
        material_coefficients: &MaterialsCoefficientsTable<N>,
        active_bodies: &[Bodies::Handle],
    ) {
        /*
         *
         * Collect active joints.
         *
         */
        let mut active_joints = Vec::new();
        constraints.foreach(|h, j| {
            if j.is_active(bodies) {
                active_joints.push(h)
            }
        });

        /*
         *
         * Collect contact manifolds.
         *
         */
        let mut contact_manifolds = Vec::new(); // FIXME: avoid allocations.
        for (h1, c1, h2, c2, _, manifold) in gworld.contact_pairs(colliders, false) {
            let b1 = try_continue!(bodies.get(c1.body()));
            let b2 = try_continue!(bodies.get(c2.body()));

            if manifold.len() > 0
                && b1.status() != BodyStatus::Disabled && b2.status() != BodyStatus::Disabled
                && ((b1.status_dependent_ndofs() != 0 && b1.is_active())
                || (b2.status_dependent_ndofs() != 0 && b2.is_active()))
            {
                contact_manifolds.push(ColliderContactManifold::new(h1, c1, h2, c2, manifold));
            }
        }

        /*
         *
         * Solve the system and integrate.
         *
         */
        bodies.foreach_mut(|_, b| {
            // FIXME This is currently needed by the solver because otherwise
            // some kinematic bodies may end up with a companion_id (used as
            // an assembly_id) that it out of bounds of the velocity vector.
            // Note sure what the best place for this is though.
            b.set_companion_id(0);
        });

        counters.solver_started();
        self.step(
            counters,
            bodies,
            colliders,
            constraints,
            &contact_manifolds[..],
            &active_bodies[..],
            &active_joints[..],
            parameters,
            material_coefficients,
        );

        bodies.foreach_mut(|_, b| {
            if b.status() == BodyStatus::Kinematic {
                b.integrate(parameters)
            }
        });
        counters.solver_completed();

        /*
         *
         * Update body kinematics and dynamics
         * after the contact resolution step.
         *
         */
        // FIXME: objects involved in a non-linear position stabilization already
        // updated their kinematics.
        bodies.foreach_mut(|_, b| {
            b.update_kinematics();
            b.update_dynamics(parameters.dt());
        });
    }
}