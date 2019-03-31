use na::RealField;

/// Logical information of the geometry of a constraint.
#[derive(Copy, Clone, Debug, Default)]
pub struct ConstraintGeometry<N: RealField> {
    /// Index of the first entry of the jacobian of the constraint affecting the first body.
    pub j_id1: usize,
    /// Index of the first entry of the jacobian of the constraint affecting the second body.
    pub j_id2: usize,
    /// Index of the first entry of the constraint jacobian multiplied by the inverse mass of the first body.
    pub wj_id1: usize,
    /// Index of the first entry of the constraint jacobian multiplied by the inverse mass of the second body.
    pub wj_id2: usize,
    /// Number of degree of freedom of the first body.
    pub ndofs1: usize,
    /// Number of degree of freedom of the second body.
    pub ndofs2: usize,
    /// Scaling parameter of the SOR-prox method.
    pub r: N,
}

impl<N: RealField> ConstraintGeometry<N> {
    /// Create a costraint geometry initialized to zero.
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

    /// Return `true` if this constraint involve a body with zero degrees of freedom.
    #[inline]
    pub fn is_ground_constraint(&self) -> bool {
        self.ndofs1 == 0 || self.ndofs2 == 0
    }
}

/// A unilateral (inequality) consraint.
pub struct UnilateralConstraint<N: RealField> {
    /// The impulse applied by this constraint.
    pub impulse: N,

    /// The scaling parameter of the SOR-prox method.
    pub r: N,
    /// The target velocity change this constraint must apply.
    pub rhs: N,

    /// The index of the impulse used for its storage in an impuse cache.
    pub impulse_id: usize,

    /// The assembly index of the first body.
    pub assembly_id1: usize,
    /// The assembly index of the second body.
    pub assembly_id2: usize,

    /// Index of the first entry of the jacobian of the constraint affecting the first body.
    pub j_id1: usize,
    /// Index of the first entry of the constraint jacobian multiplied by the inverse mass of the second body.
    pub j_id2: usize,

    /// Index of the first entry of the constraint jacobian multiplied by the inverse mass of the first body.
    pub wj_id1: usize,
    /// Index of the first entry of the constraint jacobian multiplied by the inverse mass of the second body.
    pub wj_id2: usize,

    /// Number of degree of freedom of the first body.
    pub ndofs1: usize,
    /// Number of degree of freedom of the second body.
    pub ndofs2: usize,
}

impl<N: RealField> UnilateralConstraint<N> {
    /// Create a new unilateral constraint.
    #[inline]
    pub fn new(
        geom: ConstraintGeometry<N>,
        assembly_id1: usize,
        assembly_id2: usize,
        rhs: N,
        impulse: N,
        impulse_id: usize,
    ) -> Self {
        assert!(geom.ndofs1 != 0 && geom.ndofs2 != 0);
        UnilateralConstraint {
            impulse,
            r: geom.r,
            rhs,
            impulse_id,
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

/// A unilateral (inequality) constraint between a dynamic body and one without any degrees of freedom.
pub struct UnilateralGroundConstraint<N: RealField> {
    /// The impulse applied by the constraint.
    pub impulse: N,

    /// The scaling parameter used by the SOR-prox method.
    pub r: N,
    /// The target velocity change this constraint must apply.
    pub rhs: N,

    /// The index of the impulse used for its storage in an impuse cache.
    pub impulse_id: usize,
    /// The assembly index of the dynamic body.
    pub assembly_id: usize,
    /// Index of the first entry of the jacobian of the constraint affecting the dynamic body.
    pub j_id: usize,
    /// Index of the first entry of the constraint jacobian multiplied by the inverse mass of the dynamic body.
    pub wj_id: usize,
    /// Number of degree of freedom of the dynamic body.
    pub ndofs: usize,
}

impl<N: RealField> UnilateralGroundConstraint<N> {
    /// Create a new unilateral ground constraint.
    #[inline]
    pub fn new(
        geom: ConstraintGeometry<N>,
        assembly_id1: usize,
        assembly_id2: usize,
        rhs: N,
        impulse: N,
        impulse_id: usize,
    ) -> Self {
        if geom.ndofs1 == 0 {
            UnilateralGroundConstraint {
                impulse,
                r: geom.r,
                rhs,
                impulse_id,
                assembly_id: assembly_id2,
                j_id: geom.j_id2,
                wj_id: geom.wj_id2,
                ndofs: geom.ndofs2,
            }
        } else {
            UnilateralGroundConstraint {
                impulse,
                r: geom.r,
                rhs,
                impulse_id,
                assembly_id: assembly_id1,
                j_id: geom.j_id1,
                wj_id: geom.wj_id1,
                ndofs: geom.ndofs1,
            }
        }
    }
}

/// Limits of impulse applicable by a bilateral constraint.
#[derive(Copy, Clone, Debug)]
pub enum ImpulseLimits<N: RealField> {
    /// Limits that are absolute threshold.
    Independent {
        /// The lower bound of the impulse.
        min: N,
        /// The upper bound of the impulse.
        max: N,
    },
    /// Limit proportional to the impulse of another unilateral constraint.
    Dependent {
        /// Index of the unilateral constraint which this limit depends on.
        dependency: usize,
        /// The coefficient by which the dependent impulse is multiplied to obtain the impulse limit.
        coeff: N,
    },
}

/// A bilateral (equality) constraint between two bodies.
pub struct BilateralConstraint<N: RealField> {
    /// The impulse applied by this constraint.
    pub impulse: N,

    /// The scaling parameter of the SOR-prox method.
    pub r: N,
    /// The target velocity change this constraint must apply.
    pub rhs: N,

    /// Limits of impulse applicable by this constraint.
    pub limits: ImpulseLimits<N>,

    /// The index of the impulse used for its storage in an impuse cache.
    pub impulse_id: usize,

    /// The assembly index of the first body.
    pub assembly_id1: usize,
    /// The assembly index of the second body.
    pub assembly_id2: usize,

    /// Index of the first entry of the jacobian of the constraint affecting the first body.
    pub j_id1: usize,
    /// Index of the first entry of the constraint jacobian multiplied by the inverse mass of the second body.
    pub j_id2: usize,

    /// Index of the first entry of the constraint jacobian multiplied by the inverse mass of the first body.
    pub wj_id1: usize,
    /// Index of the first entry of the constraint jacobian multiplied by the inverse mass of the second body.
    pub wj_id2: usize,

    /// Number of degree of freedom of the first body.
    pub ndofs1: usize,
    /// Number of degree of freedom of the second body.
    pub ndofs2: usize,
}

impl<N: RealField> BilateralConstraint<N> {
    /// Create a new bilateral constraint.
    #[inline]
    pub fn new(
        geom: ConstraintGeometry<N>,
        assembly_id1: usize,
        assembly_id2: usize,
        limits: ImpulseLimits<N>,
        rhs: N,
        impulse: N,
        impulse_id: usize,
    ) -> Self {
        assert!(geom.ndofs1 != 0 && geom.ndofs2 != 0);
        BilateralConstraint {
            impulse,
            r: geom.r,
            rhs,
            limits,
            impulse_id,
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

/// A bilateral (equality) constraint between a dynamic body and one without any degrees of freedom.
pub struct BilateralGroundConstraint<N: RealField> {
    /// The impulse applied by the constraint.
    pub impulse: N,

    /// The scaling parameter used by the SOR-prox method.
    pub r: N,
    /// The target velocity change this constraint must apply.
    pub rhs: N,

    /// Limits of impulse applicable by this constraint.
    pub limits: ImpulseLimits<N>,

    /// The index of the impulse used for its storage in an impuse cache.
    pub impulse_id: usize,
    /// The assembly index of the dynamic body.
    pub assembly_id: usize,
    /// Index of the first entry of the jacobian of the constraint affecting the dynamic body.
    pub j_id: usize,
    /// Index of the first entry of the constraint jacobian multiplied by the inverse mass of the dynamic body.
    pub wj_id: usize,
    /// Number of degree of freedom of the dynamic body.
    pub ndofs: usize,
}

impl<N: RealField> BilateralGroundConstraint<N> {
    /// Create a new unilateral ground constraint.
    #[inline]
    pub fn new(
        geom: ConstraintGeometry<N>,
        assembly_id1: usize,
        assembly_id2: usize,
        limits: ImpulseLimits<N>,
        rhs: N,
        impulse: N,
        impulse_id: usize,
    ) -> Self {
        if geom.ndofs1 == 0 {
            BilateralGroundConstraint {
                impulse,
                r: geom.r,
                rhs,
                limits,
                impulse_id,
                assembly_id: assembly_id2,
                j_id: geom.j_id2,
                wj_id: geom.wj_id2,
                ndofs: geom.ndofs2,
            }
        } else {
            BilateralGroundConstraint {
                impulse,
                r: geom.r,
                rhs,
                limits,
                impulse_id,
                assembly_id: assembly_id1,
                j_id: geom.j_id1,
                wj_id: geom.wj_id1,
                ndofs: geom.ndofs1,
            }
        }
    }
}
