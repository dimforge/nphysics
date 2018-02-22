use na::Real;

// NOTE: strictly speaking, this is not only the geometry since it contains teh relative velocity as well.
#[derive(Copy, Clone, Debug)]
pub struct ConstraintGeometry<N: Real> {
    pub jacobian_id1: usize,
    pub jacobian_id2: usize,
    pub weighted_jacobian_id1: usize,
    pub weighted_jacobian_id2: usize,
    pub assembly_id1: usize,
    pub assembly_id2: usize,
    pub ndofs1: usize,
    pub ndofs2: usize,
    pub rhs: N,
    pub r: N,
}

impl<N: Real> ConstraintGeometry<N> {
    #[inline]
    pub fn new() -> Self {
        ConstraintGeometry {
            jacobian_id1: 0,
            jacobian_id2: 0,
            weighted_jacobian_id1: 0,
            weighted_jacobian_id2: 0,
            assembly_id1: usize::max_value(),
            assembly_id2: usize::max_value(),
            ndofs1: 0,
            ndofs2: 0,
            rhs: N::zero(),
            r: N::zero(),
        }
    }
}

pub struct UnilateralConstraint<N: Real> {
    pub impulse: N,

    pub r: N,
    pub rhs: N,

    pub assembly_id1: usize,
    pub assembly_id2: usize,

    pub jacobian_id1: usize,
    pub jacobian_id2: usize,

    pub weighted_jacobian_id1: usize,
    pub weighted_jacobian_id2: usize,

    pub ndofs1: usize,
    pub ndofs2: usize,
}

impl<N: Real> UnilateralConstraint<N> {
    #[inline]
    pub fn new(geom: ConstraintGeometry<N>) -> Self {
        assert!(geom.ndofs1 != 0 && geom.ndofs2 != 0);
        UnilateralConstraint {
            impulse: N::zero(),
            r: geom.r,
            rhs: geom.rhs,
            assembly_id1: geom.assembly_id1,
            assembly_id2: geom.assembly_id2,
            jacobian_id1: geom.jacobian_id1,
            jacobian_id2: geom.jacobian_id2,
            weighted_jacobian_id1: geom.weighted_jacobian_id1,
            weighted_jacobian_id2: geom.weighted_jacobian_id2,
            ndofs1: geom.ndofs1,
            ndofs2: geom.ndofs2,
        }
    }
}

// FIXME: find a better name?
// (To design an unilateral contact involving only one body.)
pub struct UnilateralGroundConstraint<N: Real> {
    pub impulse: N,

    pub r: N,
    pub rhs: N,

    pub assembly_id: usize,
    pub jacobian_id: usize,
    pub weighted_jacobian_id: usize,
    pub ndofs: usize,
}

impl<N: Real> UnilateralGroundConstraint<N> {
    #[inline]
    pub fn new(geom: ConstraintGeometry<N>) -> Self {
        if geom.ndofs1 == 0 {
            UnilateralGroundConstraint {
                impulse: N::zero(),
                r: geom.r,
                rhs: geom.rhs,
                assembly_id: geom.assembly_id2,
                jacobian_id: geom.jacobian_id2,
                weighted_jacobian_id: geom.weighted_jacobian_id2,
                ndofs: geom.ndofs2,
            }
        } else {
            UnilateralGroundConstraint {
                impulse: N::zero(),
                r: geom.r,
                rhs: geom.rhs,
                assembly_id: geom.assembly_id1,
                jacobian_id: geom.jacobian_id1,
                weighted_jacobian_id: geom.weighted_jacobian_id1,
                ndofs: geom.ndofs1,
            }
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub enum ImpulseLimits<N: Real> {
    Independent { min: N, max: N },
    Dependent { dependency: usize, coeff: N },
}

pub struct BilateralConstraint<N: Real> {
    pub impulse: N,

    pub r: N,
    pub rhs: N,

    pub limits: ImpulseLimits<N>,

    pub assembly_id1: usize,
    pub assembly_id2: usize,

    pub jacobian_id1: usize,
    pub jacobian_id2: usize,

    pub weighted_jacobian_id1: usize,
    pub weighted_jacobian_id2: usize,

    pub ndofs1: usize,
    pub ndofs2: usize,
}

impl<N: Real> BilateralConstraint<N> {
    #[inline]
    pub fn new(geom: ConstraintGeometry<N>, limits: ImpulseLimits<N>) -> Self {
        assert!(geom.ndofs1 != 0 && geom.ndofs2 != 0);
        BilateralConstraint {
            impulse: N::zero(),
            r: geom.r,
            rhs: geom.rhs,
            limits: limits,
            assembly_id1: geom.assembly_id1,
            assembly_id2: geom.assembly_id2,
            jacobian_id1: geom.jacobian_id1,
            jacobian_id2: geom.jacobian_id2,
            weighted_jacobian_id1: geom.weighted_jacobian_id1,
            weighted_jacobian_id2: geom.weighted_jacobian_id2,
            ndofs1: geom.ndofs1,
            ndofs2: geom.ndofs2,
        }
    }
}

// FIXME: find a better name?
pub struct BilateralGroundConstraint<N: Real> {
    pub impulse: N,

    pub r: N,
    pub rhs: N,

    pub limits: ImpulseLimits<N>,

    pub assembly_id: usize,
    pub jacobian_id: usize,
    pub weighted_jacobian_id: usize,
    pub ndofs: usize,
}

impl<N: Real> BilateralGroundConstraint<N> {
    #[inline]
    pub fn new(geom: ConstraintGeometry<N>, limits: ImpulseLimits<N>) -> Self {
        if geom.ndofs1 == 0 {
            BilateralGroundConstraint {
                impulse: N::zero(),
                r: geom.r,
                rhs: geom.rhs,
                limits: limits,
                assembly_id: geom.assembly_id2,
                jacobian_id: geom.jacobian_id2,
                weighted_jacobian_id: geom.weighted_jacobian_id2,
                ndofs: geom.ndofs2,
            }
        } else {
            BilateralGroundConstraint {
                impulse: N::zero(),
                r: geom.r,
                rhs: geom.rhs,
                limits: limits,
                assembly_id: geom.assembly_id1,
                jacobian_id: geom.jacobian_id1,
                weighted_jacobian_id: geom.weighted_jacobian_id1,
                ndofs: geom.ndofs1,
            }
        }
    }
}
