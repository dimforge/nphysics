use std::borrow;
use ncollide::bounding_volume::{HasBoundingVolume, AABB};
use object::{RigidBody, SoftBody};

pub enum Body {
    RB(RigidBody),
    SB(SoftBody) // FIXME

}

impl Clone for Body {
    fn clone(&self) -> Body {
        match *self {
            RB(ref rb) => RB(rb.clone()),
            SB(ref sb) => SB(sb.clone())
        }
    }
}

impl Body {
    #[inline]
    pub fn to_rigid_body_or_fail<'r>(&'r self) -> &'r RigidBody {
        match *self {
            RB(ref rb) => rb,
            SB(_) => fail!("This is a SoftBody, not a RigidBody.")
        }
    }

    #[inline]
    pub fn to_mut_rigid_body_or_fail<'r>(&'r mut self) -> &'r mut RigidBody {
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

impl Eq for Body {
    #[inline]
    fn eq(&self, other: &Body) -> bool {
        borrow::ref_eq(self, other)
    }
}

impl HasBoundingVolume<AABB> for Body {
    #[inline]
    fn bounding_volume(&self) -> AABB {
        match *self {
            RB(ref rb) => rb.bounding_volume(),
            SB(_)   => fail!("Not yet implemented."),
        }
    }
}
