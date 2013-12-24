use nalgebra::na::{Vec3, Mat3, Iso3};
use ncollide::broad::DBVTBroadPhase;
use ncollide::narrow::GeomGeomCollisionDetector;
use integration::{BodyForceGenerator, BodySmpEulerIntegrator, BodyDamping, SweptBallMotionClamping};
use detection::collision::bodies_bodies::{BodyBodyDispatcher, BodiesBodies};
use detection::constraint::Constraint;
use detection::JointManager;
use detection::joint::ball_in_socket::BallInSocket;
use detection::joint::fixed::Fixed;
use detection::IslandActivationManager;
use resolution::constraint::accumulated_impulse_solver::AccumulatedImpulseSolver;
use world::{World, BodyWorld};
use object::{RigidBody, Body};

pub use ncollide::aliases::dim3::*;

type LV<N> = Vec3<N>;
type AV<N> = Vec3<N>;
type II<N> = Mat3<N>;
type M<N>  = Iso3<N>;

// fancier names
pub type Transform3d<N>       = M<N>;
pub type LinearVelocity3d<N>  = LV<N>;
pub type AngularVelocity3d<N> = AV<N>;
pub type InertiaTensor3d<N>   = II<N>;

pub type ForceGenerator3d<N> = BodyForceGenerator<N, LV<N>, AV<N>, M<N>, II<N>>;
pub type BodyIntegrator3d<N> = BodySmpEulerIntegrator<N, LV<N>, AV<N>, M<N>, II<N>>;
pub type BodyDamping3d<N> = BodyDamping<N, LV<N>, AV<N>, M<N>, II<N>>;

pub type BodyBodyDispatcher3d<N> = BodyBodyDispatcher<N, LV<N>, AV<N>, M<N>, II<N>>;
pub type DBVTBroadPhase3d<N> = DBVTBroadPhase<N, LV<N>, Body3d<N>, AABB3d<N>, BodyBodyDispatcher3d<N>, GeomGeomCollisionDetector3d<N>>;
pub type GeomGeomCollisionDetector3d<N> = ~GeomGeomCollisionDetector<N, LV<N>, M<N>, AV<N>, II<N>>;
pub type DBVTCollisionDetector3d<N> = BodiesBodies<N, LV<N>, AV<N>, M<N>, II<N>, DBVTBroadPhase3d<N>>;
pub type DBVTSweptBallMotionClamping3d<N> = SweptBallMotionClamping<N, LV<N>, AV<N>, M<N>, II<N>, DBVTBroadPhase3d<N>>;
pub type JointManager3d<N> = JointManager<N, LV<N>, AV<N>, M<N>, II<N>>;
pub type IslandActivationManager3d<N> = IslandActivationManager<N, LV<N>, AV<N>, M<N>, II<N>>; 

pub type ContactSolver3d<N> = AccumulatedImpulseSolver<N, LV<N>, AV<N>, M<N>, II<N>, Mat3<N>>;

pub type Constraint3d<N> = Constraint<N, LV<N>, AV<N>, M<N>, II<N>>;
pub type RigidBody3d<N> = RigidBody<N, LV<N>, AV<N>, M<N>, II<N>>; 
pub type Body3d<N> = Body<N, LV<N>, AV<N>, M<N>, II<N>>; 

pub type World3d<N> = World<N, Body3d<N>, Constraint3d<N>>;
pub type BodyWorld3d<N> = BodyWorld<N, LV<N>, AV<N>, M<N>, II<N>, Mat3<N>>;

/*
 * Joints
 */
pub type BallInSocket3d<N> = BallInSocket<N, LV<N>, AV<N>, M<N>, II<N>>;
pub type Fixed3d<N> = Fixed<N, LV<N>, AV<N>, M<N>, II<N>>;
