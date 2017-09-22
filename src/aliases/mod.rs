//! Aliases for complicated parameterized types.

use ncollide::bounding_volume::AABB;
use ncollide::broad_phase::DBVTBroadPhase;
// use integration::SweptBallMotionClamping;
use object::RigidBody;
use math::Point;

/// The type of the broad phase used by the world by default.
pub type DefaultBroadPhase<N> = DBVTBroadPhase<Point<N>, ::Rc<RigidBody<N>>, AABB<Point<N>>>;
