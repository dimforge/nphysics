use na::{Real, Unit};
use ncollide::query::ContactKinematic;

use object::BodyHandle;
use math::{Point, Vector};

pub struct NonlinearUnilateralConstraint<N: Real> {
    pub inv_r: N,
    pub rhs: N,

    pub ndofs1: usize,
    pub body1: BodyHandle,

    pub ndofs2: usize,
    pub body2: BodyHandle,

    pub kinematic: ContactKinematic<Vector<N>>,

    pub local1: Point<N>,
    pub normal1: Unit<Vector<N>>,
    pub margin1: N,

    pub local2: Point<N>,
    pub normal2: Unit<Vector<N>>,
    pub margin2: N,
}

impl<N: Real> NonlinearUnilateralConstraint<N> {
    pub fn new(
        body1: BodyHandle,
        ndofs1: usize,
        body2: BodyHandle,
        ndofs2: usize,
        local1: Point<N>,
        normal1: Unit<Vector<N>>,
        margin1: N,
        local2: Point<N>,
        normal2: Unit<Vector<N>>,
        margin2: N,
        kinematic: ContactKinematic<Vector<N>>,
    ) -> Self {
        let inv_r = N::zero();
        let rhs = N::zero();

        NonlinearUnilateralConstraint {
            inv_r,
            rhs,
            ndofs1,
            body1,
            ndofs2,
            body2,
            kinematic,
            local1,
            normal1,
            margin1,
            local2,
            normal2,
            margin2,
        }
    }
}

pub struct NormalConeConsraint<V: Vector> {
    pub inv_r: V::Real,
    pub rhs: V::Real,

    pub ndofs1: usize,
    pub body1: BodyHandle,

    pub ndofs2: usize,
    pub body2: BodyHandle,

    pub ncone1: PolyhedralCone<Vector>,
    pub ncone2: PolyhedralCone<Vector>,
}

impl<V: Vector> NormalConeConsraint<V> {
    pub fn new(
        body1: BodyHandle,
        ndofs1: usize,
        body2: BodyHandle,
        ndofs2: usize,
        ncone1: PolyhedralCone<Vector>,
        ncone2: PolyhedralCone<Vector>,
    ) -> Self {
        let inv_r = na::zero();
        let rhs = na::zero();

        NormalConeConsraint {
            inv_r,
            rhs,
            body1,
            ndofs1,
            body2,
            ndofs2,
            ncone1,
            ncone2,
        }
    }
}
