use nalgebra::na;
use nalgebra::structs::mat::Mat3MulRhs;
use nalgebra::na::{Vec3, Mat3, Iso3};
use ncollide::geom::{Geom, Ball, Plane, Box, Cylinder, Cone};
use ncollide::bounding_volume::AABB;
use ncollide::broad::DBVTBroadPhase;
use integration::{BodyForceGenerator, BodySmpEulerIntegrator, BodyDamping, SweptBallMotionClamping};
use detection::collision::bodies_bodies::{Dispatcher, PairwiseDetector, BodiesBodies};
use detection::constraint::Constraint;
use detection::JointManager;
use detection::joint::ball_in_socket::BallInSocket;
use detection::joint::fixed::Fixed;
use detection::IslandActivationManager;
use resolution::constraint::accumulated_impulse_solver::AccumulatedImpulseSolver;
use world::{World, BodyWorld};
use object::volumetric::InertiaTensor;
use object::{RigidBody, Body};

type LV<N> = Vec3<N>;
type AV<N> = Vec3<N>;
type II<N> = Mat3<N>;
type M<N>  = Iso3<N>;

// fancier names
pub type Transform3d<N>       = M<N>;
pub type LinearVelocity3d<N>  = LV<N>;
pub type AngularVelocity3d<N> = AV<N>;
pub type InertiaTensor3d<N>   = II<N>;

pub type Ball3d<N>     = Ball<N>;
pub type Box3d<N>      = Box<N, Vec3<N>>;
pub type Cylinder3d<N> = Cylinder<N>;
pub type Cone3d<N>     = Cone<N>;
pub type Plane3d<N>    = Plane<N, Vec3<N>>;
pub type Geom3d<N>     = Geom<N, LV<N>, M<N>>;
pub type AABB3d<N>     = AABB<N, LV<N>>;

pub type ForceGenerator3d<N> = BodyForceGenerator<N, LV<N>, AV<N>, M<N>, II<N>>;
pub type BodyIntegrator3d<N> = BodySmpEulerIntegrator<N, LV<N>, AV<N>, M<N>, II<N>>;
pub type BodyDamping3d<N> = BodyDamping<N, LV<N>, AV<N>, M<N>, II<N>>;

pub type Dispatcher3d<N> = Dispatcher<N, LV<N>, AV<N>, M<N>, II<N>>;
pub type DBVTBroadPhase3d<N> = DBVTBroadPhase<N, LV<N>, Body3d<N>, AABB3d<N>, Dispatcher3d<N>, PairwiseDetector3d<N>>;
pub type PairwiseDetector3d<N> = PairwiseDetector<N, LV<N>, AV<N>, M<N>>;
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

/// NOTE: it is a bit unfortunate to have to specialize that for the raw types.
impl<N: Num + Algebraic + Clone + Mat3MulRhs<N, Mat3<N>>>
InertiaTensor<N, LV<N>, AV<N>, Transform3d<N>> for InertiaTensor3d<N> {
    #[inline]
    fn apply(&self, av: &AV<N>) -> AV<N> {
        *self * *av
    }
    #[inline]
    fn to_world_space(&self, t: &Transform3d<N>) -> InertiaTensor3d<N> {
        let inv = na::inv(&t.rotation).unwrap();
        *t.rotation.submat() * *self * *inv.submat()
    }

    #[inline]
    fn to_relative_wrt_point(&self, mass: &N, pt: &LV<N>) -> InertiaTensor3d<N> {
        let diag  = na::sqnorm(pt);
        let diagm = Mat3::new(
            diag.clone(), na::zero(),   na::zero(),
            na::zero(),   diag.clone(), na::zero(),
            na::zero(),   na::zero(),   diag
        );

        *self + (diagm - na::outer(pt, pt)) * *mass
    }
}
