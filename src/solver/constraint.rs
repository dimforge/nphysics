use na::Real;

#[derive(Copy, Clone, Debug)]
pub struct ConstraintGeometry<N: Real> {
    pub j_id1: usize,
    pub j_id2: usize,
    pub wj_id1: usize,
    pub wj_id2: usize,
    pub ndofs1: usize,
    pub ndofs2: usize,
    pub r: N,
}

impl<N: Real> ConstraintGeometry<N> {
    #[inline]
    pub fn new() -> Self {
        ConstraintGeometry {
            j_id1: 0,
            j_id2: 0,
            wj_id1: 0,
            wj_id2: 0,
            ndofs1: 0,
            ndofs2: 0,
            r: N::zero(),
        }
    }

    #[inline]
    pub fn is_ground_constraint(&self) -> bool {
        self.ndofs1 == 0 || self.ndofs2 == 0
    }
}

pub struct UnilateralConstraint<N: Real> {
    pub impulse: N,

    pub r: N,
    pub rhs: N,

    pub cache_id: usize,

    pub assembly_id1: usize,
    pub assembly_id2: usize,

    pub j_id1: usize,
    pub j_id2: usize,

    pub wj_id1: usize,
    pub wj_id2: usize,

    pub ndofs1: usize,
    pub ndofs2: usize,
}

impl<N: Real> UnilateralConstraint<N> {
    #[inline]
    pub fn new(geom: ConstraintGeometry<N>, assembly_id1: usize, assembly_id2: usize, rhs: N, impulse: N, cache_id: usize) -> Self {
        assert!(geom.ndofs1 != 0 && geom.ndofs2 != 0);
        UnilateralConstraint {
            impulse: impulse,
            r: geom.r,
            rhs: rhs,
            cache_id: cache_id,
            assembly_id1: assembly_id1,
            assembly_id2: assembly_id2,
            j_id1: geom.j_id1,
            j_id2: geom.j_id2,
            wj_id1: geom.wj_id1,
            wj_id2: geom.wj_id2,
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

    pub cache_id: usize,
    pub assembly_id: usize,
    pub j_id: usize,
    pub wj_id: usize,
    pub ndofs: usize,
}

impl<N: Real> UnilateralGroundConstraint<N> {
    #[inline]
    pub fn new(geom: ConstraintGeometry<N>, assembly_id1: usize, assembly_id2: usize, rhs: N, impulse: N, cache_id: usize) -> Self {
        if geom.ndofs1 == 0 {
            UnilateralGroundConstraint {
                impulse: impulse,
                r: geom.r,
                rhs: rhs,
                cache_id: cache_id,
                assembly_id: assembly_id2,
                j_id: geom.j_id2,
                wj_id: geom.wj_id2,
                ndofs: geom.ndofs2,
            }
        } else {
            UnilateralGroundConstraint {
                impulse: impulse,
                r: geom.r,
                rhs: rhs,
                cache_id: cache_id,
                assembly_id: assembly_id1,
                j_id: geom.j_id1,
                wj_id: geom.wj_id1,
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

    pub cache_id: usize,

    pub assembly_id1: usize,
    pub assembly_id2: usize,

    pub j_id1: usize,
    pub j_id2: usize,

    pub wj_id1: usize,
    pub wj_id2: usize,

    pub ndofs1: usize,
    pub ndofs2: usize,
}

impl<N: Real> BilateralConstraint<N> {
    #[inline]
    pub fn new(
        geom: ConstraintGeometry<N>,
        assembly_id1: usize,
        assembly_id2: usize,
        limits: ImpulseLimits<N>,
        rhs: N,
        impulse: N,
        cache_id: usize,
    ) -> Self {
        assert!(geom.ndofs1 != 0 && geom.ndofs2 != 0);
        BilateralConstraint {
            impulse: impulse,
            r: geom.r,
            rhs: rhs,
            limits: limits,
            cache_id: cache_id,
            assembly_id1: assembly_id1,
            assembly_id2: assembly_id2,
            j_id1: geom.j_id1,
            j_id2: geom.j_id2,
            wj_id1: geom.wj_id1,
            wj_id2: geom.wj_id2,
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

    pub cache_id: usize,
    pub assembly_id: usize,
    pub j_id: usize,
    pub wj_id: usize,
    pub ndofs: usize,
}

impl<N: Real> BilateralGroundConstraint<N> {
    #[inline]
    pub fn new(
        geom: ConstraintGeometry<N>,
        assembly_id1: usize,
        assembly_id2: usize,
        limits: ImpulseLimits<N>,
        rhs: N,
        impulse: N,
        cache_id: usize,
    ) -> Self {
        if geom.ndofs1 == 0 {
            BilateralGroundConstraint {
                impulse: impulse,
                r: geom.r,
                rhs: rhs,
                limits: limits,
                cache_id: cache_id,
                assembly_id: assembly_id2,
                j_id: geom.j_id2,
                wj_id: geom.wj_id2,
                ndofs: geom.ndofs2,
            }
        } else {
            BilateralGroundConstraint {
                impulse: impulse,
                r: geom.r,
                rhs: rhs,
                limits: limits,
                cache_id: cache_id,
                assembly_id: assembly_id1,
                j_id: geom.j_id1,
                wj_id: geom.wj_id1,
                ndofs: geom.ndofs1,
            }
        }
    }
}
