use na::{DVector, Real, Unit};

use object::{BodyHandle, BodySet};
use solver::{ConstraintSet, IntegrationParameters, NonlinearConstraintGenerator, GenericNonlinearConstraint};
use solver::helper;
use joint::JointConstraint;
use math::{AngularVector, Point, Vector, SPATIAL_DIM};

pub struct RevoluteConstraint<N: Real> {
    b1: BodyHandle,
    b2: BodyHandle,
    anchor1: Point<N>,
    anchor2: Point<N>,
    axis1: Unit<AngularVector<N>>, // FIXME: not needed in 2D.
    axis2: Unit<AngularVector<N>>, // FIXME: not needed in 2D.
    lin_impulses: Vector<N>,

    min_angle: Option<N>,
    max_angle: Option<N>,
}

impl<N: Real> RevoluteConstraint<N> {
    #[cfg(feature = "dim3")]
    pub fn new(
        b1: BodyHandle,
        b2: BodyHandle,
        anchor1: Point<N>,
        axis1: Unit<AngularVector<N>>,
        anchor2: Point<N>,
        axis2: Unit<AngularVector<N>>,
    ) -> Self {
        let min_angle = None;
        let max_angle = None;
        RevoluteConstraint {
            b1,
            b2,
            anchor1,
            anchor2,
            axis1,
            axis2,
            lin_impulses: Vector::zeros(),
            min_angle,
            max_angle,
        }
    }

    #[cfg(feature = "dim2")]
    pub fn new(b1: BodyHandle, b2: BodyHandle, anchor1: Point<N>, anchor2: Point<N>) -> Self {
        let min_angle = None;
        let max_angle = None;
        let axis1 = AngularVector::x_axis();
        let axis2 = AngularVector::x_axis();

        RevoluteConstraint {
            b1,
            b2,
            anchor1,
            anchor2,
            axis1,
            axis2,
            lin_impulses: Vector::zeros(),
            min_angle,
            max_angle,
        }
    }

    pub fn min_angle(&self) -> Option<N> {
        self.min_angle
    }

    pub fn max_angle(&self) -> Option<N> {
        self.max_angle
    }

    pub fn disable_min_angle(&mut self) {
        self.min_angle = None;
    }

    pub fn disable_max_angle(&mut self) {
        self.max_angle = None;
    }

    pub fn enable_min_angle(&mut self, limit: N) {
        self.min_angle = Some(limit);
        self.assert_limits();
    }

    pub fn enable_max_angle(&mut self, limit: N) {
        self.max_angle = Some(limit);
        self.assert_limits();
    }

    fn assert_limits(&self) {
        if let (Some(min_angle), Some(max_angle)) = (self.min_angle, self.max_angle) {
            assert!(
                min_angle <= max_angle,
                "RevoluteJoint constraint limits: the min angle must be larger than (or equal to) the max angle.");
        }
    }
}

impl<N: Real> JointConstraint<N> for RevoluteConstraint<N> {
    fn num_velocity_constraints(&self) -> usize {
        SPATIAL_DIM - 1
    }

    fn anchors(&self) -> (BodyHandle, BodyHandle) {
        (self.b1, self.b2)
    }

    fn velocity_constraints(
        &mut self,
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
            &self.lin_impulses,
            0,
            ground_j_id,
            j_id,
            jacobians,
            constraints,
        );

        #[cfg(feature = "dim3")]
        {
            let axis1 = pos1 * self.axis1;
            let axis2 = pos2 * self.axis2;

            helper::restrict_relative_angular_velocity_to_axis(
                params,
                &b1,
                &b2,
                assembly_id1,
                assembly_id2,
                &axis1,
                &axis2,
                &anchor1,
                &anchor2,
                ext_vels,
                ground_j_id,
                j_id,
                jacobians,
                constraints,
            );
        }

        /*
         *
         * Limit constraints.
         *
         */
    }
        
    fn cache_impulses(&mut self, constraints: &ConstraintSet<N>) {
    }
}



impl<N: Real> NonlinearConstraintGenerator<N> for RevoluteConstraint<N> {
    fn num_position_constraints(&self, _: &BodySet<N>) -> usize {
        0
    }

    fn position_constraint(
        &self,
        params: &IntegrationParameters<N>,
        i: usize,
        bodies: &mut BodySet<N>,
        jacobians: &mut [N],
    ) -> Option<GenericNonlinearConstraint<N>> {
        None
    }
}
