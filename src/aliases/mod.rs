use ncollide::broad::DBVTBroadPhase;
use ncollide::narrow::GeomGeomCollisionDetector;
use ncollide::bounding_volume::AABB;
use integration::SweptBallMotionClamping;
use detection::collision::bodies_bodies::{BodyBodyDispatcher, BodiesBodies};
use object::Body;

pub type DefaultBroadPhase = DBVTBroadPhase<Body, AABB, BodyBodyDispatcher, ~GeomGeomCollisionDetector>;
pub type DefaultCollisionDetector = BodiesBodies<DBVTBroadPhase<Body, AABB, BodyBodyDispatcher, ~GeomGeomCollisionDetector>>;
pub type DefaultSweptBallMotionClamping = SweptBallMotionClamping<DBVTBroadPhase<Body, AABB, BodyBodyDispatcher, ~GeomGeomCollisionDetector>>;
