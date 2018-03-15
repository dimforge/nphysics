use na::{DVector, Real};

use object::{BodyHandle, BodySet};
use solver::{ConstraintSet, GenericNonlinearConstraint, IntegrationParameters};
use solver::helper;
use joint::ConstraintGenerator;
use math::{Point, DIM};

pub struct BallConstraint<N: Real> {
    b1: BodyHandle,
    b2: BodyHandle,
    anchor1: Point<N>,
    anchor2: Point<N>,
}

impl<N: Real> BallConstraint<N> {
    pub fn new(b1: BodyHandle, b2: BodyHandle, anchor1: Point<N>, anchor2: Point<N>) -> Self {
        BallConstraint {
            b1,
            b2,
            anchor1,
            anchor2,
        }
    }

    pub fn set_anchor_1(&mut self, anchor1: Point<N>) {
        self.anchor1 = anchor1;
    }

    pub fn set_anchor_2(&mut self, anchor2: Point<N>) {
        self.anchor2 = anchor2;
    }
}

impl<N: Real> ConstraintGenerator<N> for BallConstraint<N> {
    fn nconstraints(&self) -> usize {
        DIM
    }

    fn anchors(&self) -> (BodyHandle, BodyHandle) {
        (self.b1, self.b2)
    }

    fn build_constraints(
        &self,
        params: &IntegrationParameters<N>,
        bodies: &BodySet<N>,
        ext_vels: &DVector<N>,
        ground_j_id: &mut usize,
        j_id: &mut usize,
        jacobians: &mut [N],
        constraints: &mut ConstraintSet<N>,
    ) {
        let b1 = bodies.body_part(self.b1);
        let b2 = bodies.body_part(self.b2);

        /*
         *
         * Joint constraints.
         *
         */
        let pos1 = b1.position();
        let pos2 = b2.position();

        let anchor1 = pos1 * self.anchor1;
        let anchor2 = pos2 * self.anchor2;

        let assembly_id1 = b1.parent_companion_id();
        let assembly_id2 = b2.parent_companion_id();

        helper::cancel_relative_linear_velocity(
            params,
            &b1,
            &b2,
            assembly_id1,
            assembly_id2,
            &anchor1,
            &anchor2,
            ext_vels,
            ground_j_id,
            j_id,
            jacobians,
            constraints,
        );
    }
}

// impl<N: Real> NonlinearConstraintGenerator<N> for BallConstraint<N> {
//     fn nconstraints(&self, bodies: &BodySet<N>) -> usize {
//         1
//     }

//     fn constraint(
//         &self,
//         params: &IntegrationParameters<N>,
//         i: usize,
//         bodies: &mut BodySet<N>,
//         jacobians: &mut [N],
//     ) -> Option<GenericNonlinearConstraint<N>> {
//         let body1 = bodies.body_part(self.b1);
//         let body2 = bodies.body_part(self.b2);

//         /*
//          *
//          * Joint constraints.
//          *
//          */
//         let pos1 = body1.position();
//         let pos2 = body2.position();

//         let anchor1 = pos1 * self.anchor1;
//         let anchor2 = pos2 * self.anchor2;

//         let assembly_id1 = body1.parent_companion_id();
//         let assembly_id2 = body2.parent_companion_id();

//         cancel_relative_translation<N: Real>(
//             params,
//             &body1,
//             &body2,
//             assembly_id1: usize,
//             assembly_id2: usize,
//             anchor1: &Point<N>,
//             anchor2: &Point<N>,
//             ext_vels: &DVector<N>,
//             jacobians: &mut [N],
//         )
//     }
// }
