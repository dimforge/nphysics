use std::managed;
use std::num::Zero;
use nalgebra::mat::{Translation, Rotate, Transform};
use nalgebra::vec::AlgebraicVecExt;
use ncollide::bounding_volume::{HasBoundingVolume, AABB};
use object::{RigidBody, SoftBody};

// FIXME: #[deriving(ToStr, Clone, DeepClone)]
pub enum Body<N, LV, AV, M, II> {
    RB(@mut RigidBody<N, LV, AV, M, II>),
    SB(@mut SoftBody<N, LV>) // FIXME

}

impl<N: Clone, LV, AV, M, II> Body<N, LV, AV, M, II> {
    pub fn is_active(&self) -> bool {
        match *self {
            RB(rb) => rb.is_active(),
            SB(sb)  => sb.is_active()
        }
    }

    pub fn can_move(&self) -> bool {
        match *self {
            RB(rb) => rb.can_move(),
            SB(_)   => true
        }
    }

    pub fn index(&self) -> int {
        match *self {
            RB(rb) => rb.index(),
            SB(sb)  => sb.index()
        }
    }

    pub fn set_index(&mut self, index: int) {
        match *self {
            RB(rb) => rb.set_index(index),
            SB(sb)  => sb.set_index(index)
        }
    }

    pub fn activate(&mut self) {
        match *self {
            RB(rb) => rb.activate(),
            SB(sb)  => sb.activate()
        }
    }
}

impl<N, LV: Zero, AV: Zero, M, II> Body<N, LV, AV, M, II> {
    pub fn deactivate(&mut self) {
        match *self {
            RB(rb) => rb.deactivate(),
            SB(sb)  => sb.deactivate()
        }
    }
}

impl<N, LV, AV, M, II> Clone for Body<N, LV, AV, M, II> {
    fn clone(&self) -> Body<N, LV, AV, M, II> {
        match *self {
            RB(rb)  => RB(rb),
            SB(sb)  => SB(sb)
        }
    }
}

impl<N, LV, AV, M, II> Eq for Body<N, LV, AV, M, II> {
    fn eq(&self, other: &Body<N, LV, AV, M, II>) -> bool {
        match (*self, *other) {
            (RB(rb1), RB(rb2)) => managed::mut_ptr_eq(rb1, rb2), 
            (SB(sb1) , SB(sb2))  => managed::mut_ptr_eq(sb1, sb2), 
            _                                => false
        }
    }
}

impl<N:  NumCast + Primitive + Orderable + Algebraic + Signed + Clone + ToStr,
     LV: AlgebraicVecExt<N> + Clone + ToStr,
     AV,
     M:  Translation<LV> + Rotate<LV> + Transform<LV> + Mul<M, M>,
     II>
HasBoundingVolume<AABB<N, LV>> for Body<N, LV, AV, M, II> {
    #[inline]
    fn bounding_volume(&self) -> AABB<N, LV> {
        match *self {
            RB(rb) => rb.bounding_volume(),
            SB(_)   => fail!("Not yet implemented."),
        }
    }
}

pub trait ToRigidBody<N, LV, AV, M, II> {
    fn to_rigid_body(&self) -> Option<@mut RigidBody<N, LV, AV, M, II>>;
    fn to_rigid_body_or_fail(&self) -> @mut RigidBody<N, LV, AV, M, II>;
}

impl<N, LV, AV, M, II> ToRigidBody<N, LV, AV, M, II>
for Body<N, LV, AV, M, II> {
    fn to_rigid_body(&self) -> Option<@mut RigidBody<N, LV, AV, M, II>> {
        match *self {
            RB(rb) => Some(rb),
            _      => None
        }
    }

    fn to_rigid_body_or_fail(&self) -> @mut RigidBody<N, LV, AV, M, II> {
        match *self {
            RB(rb) => rb,
            _      => fail!("Cannot extract a rigid body from this body.")
        }
    }
}

pub trait ToSoftBody<N, V> {
    fn to_soft_body(&self) -> Option<@mut SoftBody<N, V>>;
    fn to_soft_body_or_fail(&self) -> @mut SoftBody<N, V>;
}

impl<N, LV, AV, M, II> ToSoftBody<N, LV>
for Body<N, LV, AV, M, II> {
    fn to_soft_body(&self) -> Option<@mut SoftBody<N, LV>> {
        match *self {
            SB(sb) => Some(sb),
            _      => None
        }
    }

    fn to_soft_body_or_fail(&self) -> @mut SoftBody<N, LV> {
        match *self {
            SB(sb) => sb,
            _      => fail!("Cannot extract a soft body from this body.")
        }
    }
}
