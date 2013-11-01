use std::borrow;
use ncollide::bounding_volume::{HasBoundingVolume, AABB};
use object::{RigidBody, SoftBody};
use aliases::traits::{NPhysicsScalar, NPhysicsDirection, NPhysicsOrientation, NPhysicsTransform,
                      NPhysicsInertia};

pub enum Body<N, LV, AV, M, II> {
    RB(RigidBody<N, LV, AV, M, II>),
    SB(SoftBody<N, LV>) // FIXME

}

impl<N:  Send + Freeze + Clone,
     LV: Send + Freeze + Clone,
     AV: Clone,
     M:  Send + Freeze + Clone,
     II: Clone>
Clone for Body<N, LV, AV, M, II> {
    fn clone(&self) -> Body<N, LV, AV, M, II> {
        match *self {
            RB(ref rb) => RB(rb.clone()),
            SB(ref sb) => SB(sb.clone())
        }
    }
}

impl<N:  Clone + NPhysicsScalar,
     LV: Clone + NPhysicsDirection<N, AV>,
     AV: Clone + NPhysicsOrientation<N>,
     M:  NPhysicsTransform<LV, AV>,
     II: Clone + NPhysicsInertia<N, LV, AV, M>>
Body<N, LV, AV, M, II> {
    #[inline]
    pub fn to_rigid_body_or_fail<'r>(&'r self) -> &'r RigidBody<N, LV, AV, M, II> {
        match *self {
            RB(ref rb) => rb,
            SB(_) => fail!("This is a SoftBody, not a RigidBody.")
        }
    }

    #[inline]
    pub fn to_mut_rigid_body_or_fail<'r>(&'r mut self) -> &'r mut RigidBody<N, LV, AV, M, II> {
        match *self {
            RB(ref mut rb) => rb,
            SB(_) => fail!("This is a SoftBody, not a RigidBody.")
        }
    }

    #[inline]
    pub fn is_active(&self) -> bool {
        match *self {
            RB(ref rb) => rb.is_active(),
            SB(ref sb) => sb.is_active()
        }
    }

    #[inline]
    pub fn can_move(&self) -> bool {
        match *self {
            RB(ref rb) => rb.can_move(),
            SB(_)   => true
        }
    }

    #[inline]
    pub fn index(&self) -> int {
        match *self {
            RB(ref rb) => rb.index(),
            SB(ref sb) => sb.index()
        }
    }

    #[inline]
    pub fn set_index(&mut self, index: int) {
        match *self {
            RB(ref mut rb) => rb.set_index(index),
            SB(ref mut sb)  => sb.set_index(index)
        }
    }

    #[inline]
    pub fn activate(&mut self) {
        match *self {
            RB(ref mut rb) => rb.activate(),
            SB(ref mut sb)  => sb.activate()
        }
    }

    #[inline]
    pub fn deactivate(&mut self) {
        match *self {
            RB(ref mut rb) => rb.deactivate(),
            SB(ref mut sb)  => sb.deactivate()
        }
    }
}

impl<N, LV, AV, M, II> Eq for Body<N, LV, AV, M, II> {
    #[inline]
    fn eq(&self, other: &Body<N, LV, AV, M, II>) -> bool {
        borrow::ref_eq(self, other)
    }
}

impl<N:  Clone + NPhysicsScalar,
     LV: Clone + NPhysicsDirection<N, AV>,
     AV: NPhysicsOrientation<N>,
     M:  NPhysicsTransform<LV, AV>,
     II: NPhysicsInertia<N, LV, AV, M>>
HasBoundingVolume<AABB<N, LV>> for Body<N, LV, AV, M, II> {
    #[inline]
    fn bounding_volume(&self) -> AABB<N, LV> {
        match *self {
            RB(ref rb) => rb.bounding_volume(),
            SB(_)   => fail!("Not yet implemented."),
        }
    }
}
