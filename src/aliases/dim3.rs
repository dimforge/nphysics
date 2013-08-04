use nalgebra::mat::Mat3;
use nalgebra::vec::Vec3;
use nalgebra::adaptors::transform::Transform;
use nalgebra::adaptors::rotmat::Rotmat;
use ncollide::geom::ball::Ball;
use ncollide::geom::plane::Plane;
use ncollide::geom::box::Box;
use integration::body_force_generator::BodyForceGenerator;
use integration::rigid_body_integrator::RigidBodySmpEulerIntegrator;
use detection::collision::bodies_bodies::{LBVBodiesBodies, Constraint};
use resolution::constraint::accumulated_impulse_solver::AccumulatedImpulseSolver;
use object::implicit_geom::DefaultGeom;
use object::rigid_body::RigidBody;
use object::body::Body;
use world::world::World;

type LV<N> = Vec3<N>;
type AV<N> = Vec3<N>;
type II<N> = Mat3<N>;
type M<N>  = Transform<Rotmat<Mat3<N>>, Vec3<N>>;

// fancier names
pub type Transform3d<N>       = M<N>;
pub type LinearVelocity3d<N>  = LV<N>;
pub type AngularVelocity3d<N> = AV<N>;
pub type InertiaTensor3d<N>   = II<N>;

pub type Ball3d<N>  = Ball<N, Vec3<N>>;
pub type Box3d<N>   = Box<N, Vec3<N>>;
pub type Plane3d<N> = Plane<Vec3<N>>;
pub type Geom3d<N>  = DefaultGeom<N, LV<N>, M<N>, II<N>>;

pub type ForceGenerator3d<N> = BodyForceGenerator<N, LV<N>, AV<N>, M<N>, II<N>>;
pub type RigidBodyIntegrator3d<N> = RigidBodySmpEulerIntegrator<N, LV<N>, AV<N>, M<N>, II<N>>;

pub type CollisionDetector3d<N> = LBVBodiesBodies<N, LV<N>, AV<N>, M<N>, II<N>>;

pub type ContactSolver3d<N> = AccumulatedImpulseSolver<N, LV<N>, AV<N>, M<N>, II<N>>;

pub type Constraint3d<N> = Constraint<N, LV<N>, AV<N>, M<N>, II<N>>;
pub type RigidBody3d<N> = RigidBody<N, LV<N>, AV<N>, M<N>, II<N>>; 
pub type Body3d<N> = Body<N, LV<N>, AV<N>, M<N>, II<N>>; 
pub type World3d<N> = World<N, Body3d<N>, Constraint3d<N>>;
