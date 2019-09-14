use na::{DVector, RealField, Unit};

use crate::math::{Point, Vector};
use crate::object::{Body, BodyHandle, BodyPart, BodyPartHandle};
use crate::solver::{
    helper, BilateralConstraint, BilateralGroundConstraint, ForceDirection,
    GenericNonlinearConstraint, ImpulseLimits, IntegrationParameters, LinearConstraints,
};

pub fn build_linear_limits_velocity_constraint<N: RealField, B: ?Sized + Body<N>, H: BodyHandle>(
    body1: &B,
    part1: &dyn BodyPart<N>,
    handle1: BodyPartHandle<H>,
    body2: &B,
    part2: &dyn BodyPart<N>,
    handle2: BodyPartHandle<H>,
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
    constraints: &mut LinearConstraints<N, usize>,
)
{
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

    let (ext_vels1, ext_vels2) =
        helper::split_ext_vels(body1, body2, assembly_id1, assembly_id2, ext_vels);
    let force = ForceDirection::Linear(dir);
    let mut rhs = N::zero();
    let geom = helper::constraint_pair_geometry(
        body1,
        part1,
        handle1,
        body2,
        part2,
        handle2,
        anchor1,
        anchor2,
        &force,
        ground_j_id,
        j_id,
        jacobians,
        Some(&ext_vels1),
        Some(&ext_vels2),
        Some(&mut rhs),
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
        constraints.bilateral.push(BilateralConstraint::new(
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

pub fn build_linear_limits_position_constraint<N: RealField, B: ?Sized + Body<N>, H: BodyHandle>(
    parameters: &IntegrationParameters<N>,
    body1: &B,
    part1: &dyn BodyPart<N>,
    handle1: BodyPartHandle<H>,
    body2: &B,
    part2: &dyn BodyPart<N>,
    handle2: BodyPartHandle<H>,
    anchor1: &Point<N>,
    anchor2: &Point<N>,
    axis: &Unit<Vector<N>>,
    min: Option<N>,
    max: Option<N>,
    jacobians: &mut [N],
) -> Option<GenericNonlinearConstraint<N, H>>
{
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

    if error > parameters.allowed_linear_error {
        let mut j_id = 0;
        let mut ground_j_id = 0;

        let geom = helper::constraint_pair_geometry(
            body1,
            part1,
            handle1,
            body2,
            part2,
            handle2,
            anchor1,
            anchor2,
            &ForceDirection::Linear(dir),
            &mut ground_j_id,
            &mut j_id,
            jacobians,
            None,
            None,
            None,
        );

        let rhs = -error;
        let constraint = GenericNonlinearConstraint::new(
            handle1,
            Some(handle2),
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

/*
pub fn build_angular_limit_velocity_constraint<N: RealField>(
    parameters: &IntegrationParameters<N>,
    body1: &BodyPart<N>,
    body2: &BodyPart<N>,
    pos1: &Isometry<N>,
    pos2: &Isometry<N>,
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
    constraints: &mut ConstraintSet<N, usize>,
) {
}
*/
