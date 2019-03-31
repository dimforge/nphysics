use na::{RealField, Unit};
use ncollide::query::ContactKinematic;

use crate::math::Vector;
use crate::object::{BodyPartHandle, BodySet, ColliderHandle};
use crate::solver::IntegrationParameters;

/// A generic non-linear position constraint.
pub struct GenericNonlinearConstraint<N: RealField> {
    /// The first body affected by the constraint.
    pub body1: BodyPartHandle,
    /// The second body affected by the constraint.
    pub body2: BodyPartHandle,
    /// Whether this constraint affects the bodies translation or orientation.
    pub is_angular: bool,
    //  FIXME: rename ndofs1?
    /// Number of degree of freedom of the first body.
    pub dim1: usize,
    /// Number of degree of freedom of the second body.
    pub dim2: usize,
    /// Index of the first entry of the constraint jacobian multiplied by the inverse mass of the first body.
    pub wj_id1: usize,
    /// Index of the first entry of the constraint jacobian multiplied by the inverse mass of the second body.
    pub wj_id2: usize,
    /// The target position change this constraint must apply.
    pub rhs: N,
    /// The scaling parameter of the SOR-prox method.
    pub r: N,
}

impl<N: RealField> GenericNonlinearConstraint<N> {
    /// Initialize a new nonlinear constraint.
    pub fn new(
        body1: BodyPartHandle,
        body2: BodyPartHandle,
        is_angular: bool,
        dim1: usize,
        dim2: usize,
        wj_id1: usize,
        wj_id2: usize,
        rhs: N,
        r: N,
    ) -> Self {
        GenericNonlinearConstraint {
            body1,
            body2,
            is_angular,
            dim1,
            dim2,
            wj_id1,
            wj_id2,
            rhs,
            r,
        }
    }
}

/// Implemented by structures that generate non-linear constraints.
pub trait NonlinearConstraintGenerator<N: RealField> {
    /// Maximum of non-linear position constraint this generater needs to output.
    fn num_position_constraints(&self, bodies: &BodySet<N>) -> usize;
    /// Generate the `i`-th position constraint of this generator.
    fn position_constraint(
        &self,
        params: &IntegrationParameters<N>,
        i: usize,
        bodies: &mut BodySet<N>,
        jacobians: &mut [N],
    ) -> Option<GenericNonlinearConstraint<N>>;
}

/// A non-linear position-based non-penetration constraint.
#[derive(Debug)]
pub struct NonlinearUnilateralConstraint<N: RealField> {
    /// The scaling parameter of the SOR-prox method.
    pub r: N,
    /// The target position change this constraint must apply.
    pub rhs: N,

    /// Number of degree of freedom of the first body.
    pub ndofs1: usize,
    /// The first body affected by the constraint.
    pub body1: BodyPartHandle,
    /// The first collider affected by the constraint.
    pub collider1: ColliderHandle,

    /// Number of degree of freedom of the second body.
    pub ndofs2: usize,
    /// The second body affected by the constraint.
    pub body2: BodyPartHandle,
    /// The second collider affected by the constraint.
    pub collider2: ColliderHandle,

    /// The kinematic information used to update the contact location.
    pub kinematic: ContactKinematic<N>,

    /// The contact normal on the local space of `self.body1`.
    pub normal1: Unit<Vector<N>>,
    /// The contact normal on the local space of `self.body1`.
    pub normal2: Unit<Vector<N>>,
}

impl<N: RealField> NonlinearUnilateralConstraint<N> {
    /// Create a new nonlinear position-based non-penetration constraint.
    pub fn new(
        body1: BodyPartHandle,
        collider1: ColliderHandle,
        ndofs1: usize,
        body2: BodyPartHandle,
        collider2: ColliderHandle,
        ndofs2: usize,
        normal1: Unit<Vector<N>>,
        normal2: Unit<Vector<N>>,
        kinematic: ContactKinematic<N>,
    ) -> Self {
        let r = N::zero();
        let rhs = N::zero();

        NonlinearUnilateralConstraint {
            r,
            rhs,
            ndofs1,
            body1,
            collider1,
            ndofs2,
            body2,
            collider2,
            kinematic,
            normal1,
            normal2,
        }
    }
}

/// A non-linear position constraint generator to enforce multibody joint limits.
pub struct MultibodyJointLimitsNonlinearConstraintGenerator;

impl MultibodyJointLimitsNonlinearConstraintGenerator {
    /// Creates the constraint generator from the given multibody link.
    pub fn new() -> Self {
        MultibodyJointLimitsNonlinearConstraintGenerator
    }
}

impl<N: RealField> NonlinearConstraintGenerator<N> for MultibodyJointLimitsNonlinearConstraintGenerator {
    fn num_position_constraints(&self, _: &BodySet<N>) -> usize {
        0
    }

    fn position_constraint(
        &self,
        _: &IntegrationParameters<N>,
        _: usize,
        _: &mut BodySet<N>,
        _: &mut [N],
    ) -> Option<GenericNonlinearConstraint<N>> {
        None
    }
}
