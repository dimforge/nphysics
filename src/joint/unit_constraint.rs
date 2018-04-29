use alga::linear::{FiniteDimInnerSpace, FiniteDimVectorSpace};
use na::{self, DVector, DVectorSlice, Real, Unit};
use std::ops::Neg;

use math::{AngularVector, Force, Point, Rotation, Vector};
use object::BodyPart;
use solver::{helper, BilateralConstraint, BilateralGroundConstraint, ConstraintGeometry,
             ConstraintSet, ForceDirection, GenericNonlinearConstraint, ImpulseLimits,
             IntegrationParameters};

pub fn build_linear_limits_velocity_constraint<N: Real>(
    params: &IntegrationParameters<N>,
    body1: &BodyPart<N>,
    body2: &BodyPart<N>,
    assembly_id1: usize,
    assembly_id2: usize,
    anchor1: &Point<N>,
    anchor2: &Point<N>,
    axis: &Unit<Vector<N>>,
    min: Option<N>,
    max: Option<N>,
    ext_vels: &DVector<N>,
    impulse: N,
    impulse_id: usize,
    ground_j_id: &mut usize,
    j_id: &mut usize,
    jacobians: &mut [N],
    constraints: &mut ConstraintSet<N>,
) {
    let offset = axis.dot(&(anchor2 - anchor1));

    let (unilateral, dir) = match (min, max) {
        (None, None) => {
            return;
        }
        (Some(min), Some(max)) => {
            if relative_eq!(min, max) {
                (false, *axis)
            } else {
                if offset <= min {
                    (true, -*axis)
                } else if offset >= max {
                    (true, *axis)
                } else {
                    return;
                }
            }
        }
        (Some(min), None) => {
            if offset <= min {
                (true, -*axis)
            } else {
                return;
            }
        }
        (None, Some(max)) => {
            if offset >= max {
                (true, *axis)
            } else {
                return;
            }
        }
    };

    let force = ForceDirection::Linear(dir);
    let geom = helper::constraint_pair_geometry(
        body1,
        body2,
        anchor1,
        anchor2,
        &force,
        ground_j_id,
        j_id,
        jacobians,
    );

    let rhs = helper::constraint_pair_velocity(
        &body1,
        &body2,
        assembly_id1,
        assembly_id2,
        anchor1,
        anchor2,
        &force,
        ext_vels,
        jacobians,
        &geom,
    );

    // FIXME: generate unilateral constraints for unilateral limits.
    let limits = if unilateral {
        ImpulseLimits::Independent {
            min: N::zero(),
            max: N::max_value(),
        }
    } else {
        ImpulseLimits::Independent {
            min: -N::max_value(),
            max: N::max_value(),
        }
    };

    if geom.ndofs1 == 0 || geom.ndofs2 == 0 {
        constraints
            .velocity
            .bilateral_ground
            .push(BilateralGroundConstraint::new(
                geom,
                assembly_id1,
                assembly_id2,
                limits,
                rhs,
                impulse,
                impulse_id,
            ));
    } else {
        constraints
            .velocity
            .bilateral
            .push(BilateralConstraint::new(
                geom,
                assembly_id1,
                assembly_id2,
                limits,
                rhs,
                impulse,
                impulse_id,
            ));
    }
}

pub fn build_linear_limits_position_constraint<N: Real>(
    params: &IntegrationParameters<N>,
    body1: &BodyPart<N>,
    body2: &BodyPart<N>,
    anchor1: &Point<N>,
    anchor2: &Point<N>,
    axis: &Unit<Vector<N>>,
    min: Option<N>,
    max: Option<N>,
    jacobians: &mut [N],
) -> Option<GenericNonlinearConstraint<N>> {
    let offset = axis.dot(&(anchor2 - anchor1));
    let mut error = N::zero();
    let mut dir = *axis;

    if let Some(min) = min {
        error = min - offset;
        dir = -*axis;
    }

    if error < N::zero() {
        if let Some(max) = max {
            error = offset - max;
            dir = *axis;
        }
    }

    if error > params.allowed_linear_error {
        let mut j_id = 0;
        let mut ground_j_id = 0;

        let geom = helper::constraint_pair_geometry(
            body1,
            body2,
            anchor1,
            anchor2,
            &ForceDirection::Linear(dir),
            &mut ground_j_id,
            &mut j_id,
            jacobians,
        );

        let rhs = -error;
        let constraint = GenericNonlinearConstraint::new(
            body1.handle(),
            body2.handle(),
            false,
            geom.ndofs1,
            geom.ndofs2,
            geom.wj_id1,
            geom.wj_id2,
            rhs,
            geom.r,
        );

        Some(constraint)
    } else {
        None
    }
}
