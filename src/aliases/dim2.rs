use nalgebra::na::{Mat1, Vec2, Vec1, Iso2};
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

pub use ncollide::aliases::dim2::*;

type LV<N> = Vec2<N>;
type AV<N> = Vec1<N>;
type II<N> = Mat1<N>;
type M<N>  = Iso2<N>;

// fancier names
pub type Transform2d<N>       = M<N>;
pub type LinearVelocity2d<N>  = LV<N>;
pub type AngularVelocity2d<N> = AV<N>;
pub type InertiaTensor2d<N>   = II<N>;

pub type ForceGenerator2d<N> = BodyForceGenerator<N, LV<N>, AV<N>, M<N>, II<N>>;
pub type BodyIntegrator2d<N> = BodySmpEulerIntegrator<N, LV<N>, AV<N>, M<N>, II<N>>;
pub type BodyDamping2d<N>    = BodyDamping<N, LV<N>, AV<N>, M<N>, II<N>>;

pub type BodyBodyDispatcher2d<N> = BodyBodyDispatcher<N, LV<N>, AV<N>, M<N>, II<N>>;
pub type DBVTBroadPhase2d<N> = DBVTBroadPhase<N, LV<N>, Body2d<N>, AABB2d<N>, BodyBodyDispatcher2d<N>, GeomGeomCollisionDetector2d<N>>;
pub type GeomGeomCollisionDetector2d<N> = ~GeomGeomCollisionDetector<N, LV<N>, M<N>, AV<N>, II<N>>;
pub type DBVTCollisionDetector2d<N> = BodiesBodies<N, LV<N>, AV<N>, M<N>, II<N>, DBVTBroadPhase2d<N>>;
pub type DBVTSweptBallMotionClamping2d<N> = SweptBallMotionClamping<N, LV<N>, AV<N>, M<N>, II<N>, DBVTBroadPhase2d<N>>;
pub type JointManager2d<N> = JointManager<N, LV<N>, AV<N>, M<N>, II<N>>;
pub type IslandActivationManager2d<N> = IslandActivationManager<N, LV<N>, AV<N>, M<N>, II<N>>; 

pub type ContactSolver2d<N> = AccumulatedImpulseSolver<N, LV<N>, AV<N>, M<N>, II<N>, Vec2<N>>;

pub type Constraint2d<N> = Constraint<N, LV<N>, AV<N>, M<N>, II<N>>;
pub type RigidBody2d<N> = RigidBody<N, LV<N>, AV<N>, M<N>, II<N>>; 
pub type Body2d<N> = Body<N, LV<N>, AV<N>, M<N>, II<N>>; 

pub type World2d<N> = World<N, Body2d<N>, Constraint2d<N>>;
pub type BodyWorld2d<N> = BodyWorld<N, LV<N>, AV<N>, M<N>, II<N>, Vec2<N>>;

/*
 * Joints
 */
pub type BallInSocket2d<N> = BallInSocket<N, LV<N>, AV<N>, M<N>, II<N>>;
pub type Fixed2d<N> = Fixed<N, LV<N>, AV<N>, M<N>, II<N>>;
