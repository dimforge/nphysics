//! Aliases for complicated parameterized types.

use std::rc::Rc;
use std::cell::RefCell;
use ncollide::broad::DBVTBroadPhase;
use ncollide::narrow::GeomGeomCollisionDetector;
use ncollide::bounding_volume::AABB;
// use integration::SweptBallMotionClamping;
use detection::{BodyBodyDispatcher, BodiesBodies};
use object::RigidBody;

pub type DefaultBroadPhase = DBVTBroadPhase<Rc<RefCell<RigidBody>>, AABB, BodyBodyDispatcher, Box<GeomGeomCollisionDetector:'static>>;
pub type DefaultCollisionDetector = BodiesBodies<DefaultBroadPhase>;
// pub type DefaultSweptBallMotionClamping = SweptBallMotionClamping<DefaultBroadPhase>;
