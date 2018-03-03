use na::{Real, Unit};
use ncollide::query::ContactKinematic;

use object::BodyHandle;
use math::{Point, Vector};

pub struct NonlinearUnilateralConstraint<N: Real> {
    pub impulse: N,

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
