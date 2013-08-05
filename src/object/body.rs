use std::managed;
use nalgebra::traits::scalar_op::{ScalarAdd, ScalarSub, ScalarDiv};
use ncollide::bounding_volume::bounding_volume::HasBoundingVolume;
use ncollide::bounding_volume::aabb::AABB;
use object::rigid_body::RigidBody;
use object::soft_body::SoftBody;

// FIXME: #[deriving(ToStr, Clone, DeepClone)]
pub enum Body<N, LV, AV, M, II> {
    RigidBody(@mut RigidBody<N, LV, AV, M, II>),
    SoftBody(@mut SoftBody<N, LV>) // FIXME

}

impl<N, LV, AV, M, II> Clone for Body<N, LV, AV, M, II> {
    fn clone(&self) -> Body<N, LV, AV, M, II> {
        match *self {
            RigidBody(rb) => RigidBody(rb),
            SoftBody(sb)  => SoftBody(sb)
        }
    }
}

impl<N, LV, AV, M, II> Eq for Body<N, LV, AV, M, II> {
    fn eq(&self, other: &Body<N, LV, AV, M, II>) -> bool {
        match (*self, *other) {
            (RigidBody(rb1), RigidBody(rb2)) => managed::mut_ptr_eq(rb1, rb2), 
            (SoftBody(sb1) , SoftBody(sb2))  => managed::mut_ptr_eq(sb1, sb2), 
            _                                => false
        }
    }
}

impl<N,
     LV: Bounded + ScalarAdd<N> + ScalarSub<N> + ScalarDiv<N> + Neg<LV> + Ord + Orderable + Clone,
     AV,
     M,
     II>
HasBoundingVolume<AABB<N, LV>> for Body<N, LV, AV, M, II> {
    #[inline]
    fn bounding_volume(&self) -> AABB<N, LV> {
        match *self {
            RigidBody(rb) => rb.bounding_volume(),
            SoftBody(_)   => fail!("Not yet implemented."),
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
            RigidBody(rb) => Some(rb),
            _             => None
        }
    }

    fn to_rigid_body_or_fail(&self) -> @mut RigidBody<N, LV, AV, M, II> {
        match *self {
            RigidBody(rb) => rb,
            _             => fail!("Cannot extract a rigid body from this body.")
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
            SoftBody(sb) => Some(sb),
            _            => None
        }
    }

    fn to_soft_body_or_fail(&self) -> @mut SoftBody<N, LV> {
        match *self {
            SoftBody(sb) => sb,
            _            => fail!("Cannot extract a soft body from this body.")
        }
    }
}
