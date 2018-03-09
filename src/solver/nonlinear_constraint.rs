use na::{Real, Unit};
use ncollide::bounding_volume::PolyhedralCone;
use ncollide::query::ContactKinematic;

use object::BodyHandle;
use math::{Point, Vector};

pub struct NonlinearUnilateralConstraint<N: Real> {
    pub inv_r: N,
    pub rhs: N,

    pub normal_constraint_id: usize,

    pub ndofs1: usize,
    pub body1: BodyHandle,

    pub ndofs2: usize,
    pub body2: BodyHandle,

    pub kinematic: ContactKinematic<Vector<N>>,

    pub local1: Point<N>,
    pub normal1: Unit<Vector<N>>,
    pub ncone1: PolyhedralCone<Vector<N>>,
    pub margin1: N,

    pub local2: Point<N>,
    pub normal2: Unit<Vector<N>>,
    pub ncone2: PolyhedralCone<Vector<N>>,
    pub margin2: N,
}

impl<N: Real> NonlinearUnilateralConstraint<N> {
    pub fn new(
        normal_constraint_id: usize,
        body1: BodyHandle,
        ndofs1: usize,
        body2: BodyHandle,
        ndofs2: usize,
        local1: Point<N>,
        normal1: Unit<Vector<N>>,
        ncone1: PolyhedralCone<Vector<N>>,
        margin1: N,
        local2: Point<N>,
        normal2: Unit<Vector<N>>,
        ncone2: PolyhedralCone<Vector<N>>,
        margin2: N,
        kinematic: ContactKinematic<Vector<N>>,
    ) -> Self {
        let inv_r = N::zero();
        let rhs = N::zero();

        NonlinearUnilateralConstraint {
            normal_constraint_id,
            inv_r,
            rhs,
            ndofs1,
            body1,
            ndofs2,
            body2,
            kinematic,
            local1,
            normal1,
            ncone1,
            margin1,
            local2,
            normal2,
            ncone2,
            margin2,
        }
    }
}

pub struct NonlinearNormalConeConstraint<N: Real> {
    pub inv_r: N,
    pub rhs: N,

    pub contact_id: usize,

    pub ncone1: PolyhedralCone<Vector<N>>,
    pub ncone2: PolyhedralCone<Vector<N>>,
}

impl<N: Real> NonlinearNormalConeConstraint<N> {
    pub fn new(
        contact_id: usize,
        ncone1: PolyhedralCone<Vector<N>>,
        ncone2: PolyhedralCone<Vector<N>>,
    ) -> Self {
        let inv_r = N::zero();
        let rhs = N::zero();

        NonlinearNormalConeConstraint {
            inv_r,
            rhs,
            contact_id,
            ncone1,
            ncone2,
        }
    }
}
