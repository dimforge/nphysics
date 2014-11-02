//! Aliases for complicated parameterized types.

use std::rc::Rc;
use std::cell::RefCell;
use ncollide::broad_phase::DBVTBroadPhase;
use ncollide::narrow_phase::ShapeShapeCollisionDetector;
use ncollide::bounding_volume::AABB;
// use integration::SweptBallMotionClamping;
use detection::{BodyBodyDispatcher, BodiesBodies};
use object::RigidBody;
use math::{Scalar, Point, Vect, Matrix, AngularInertia};

pub type DefaultBroadPhase = DBVTBroadPhase<Scalar, Point, Rc<RefCell<RigidBody>>, AABB<Point>, BodyBodyDispatcher, Box<ShapeShapeCollisionDetector<Scalar, Point, Vect, Matrix, AngularInertia> + 'static>>;
pub type DefaultCollisionDetector = BodiesBodies<DefaultBroadPhase>;
// pub type DefaultSweptBallMotionClamping = SweptBallMotionClamping<DefaultBroadPhase>;
