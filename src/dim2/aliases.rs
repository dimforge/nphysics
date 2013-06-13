use nalgebra::dim2::mat2::Mat2;
use nalgebra::dim2::vec2::Vec2;
use nalgebra::dim1::vec1::Vec1;
use nalgebra::dim1::mat1::Mat1;
use nalgebra::adaptors::rotmat::Rotmat;
use nalgebra::adaptors::transform::Transform;
use ncollide::broad::brute_force_broad_phase::BruteForceBroadPhase;
use ncollide::narrow::default_default::DefaultDefaultCollisionDetector;
use ncollide::narrow::has_geom_has_geom::HasGeomHasGeomCollisionDetector;
use ncollide::geom::default_geom::DefaultGeom;
use ncollide::geom::ball::Ball;
use ncollide::geom::plane::Plane;
use body::rigid_body::RigidBody;
use graph::island_accumulator::IslandAccumulator;
use integrator::body_gravity_integrator::BodyGravityIntegrator;
use constraint::accumulated_impulse_solver::AccumulatedImpulseSolver;
use constraint::geometric_contact_with_impulse::GeometricContactWithImpulse;
use world::world::World;

pub type Transform2d<N>       =  Transform<Rotmat<Mat2<N>>, Vec2<N>>;
pub type LinearVelocity2d<N>  = Vec2<N>;
pub type AngularVelocity2d<N> = Vec1<N>;
pub type InertiaTensor2d<N>   = Mat1<N>;
pub type Geom2d<N>            = DefaultGeom<N, LinearVelocity2d<N>>;
pub type RigidBody2d<N>       = RigidBody<Geom2d<N>,
                                          N,
                                          Transform2d<N>,
                                          LinearVelocity2d<N>,
                                          AngularVelocity2d<N>,
                                          InertiaTensor2d<N>>;

pub type Integrator2d<N> = BodyGravityIntegrator<LinearVelocity2d<N>,
                                                 AngularVelocity2d<N>,
                                                 N,
                                                 RigidBody2d<N>,
                                                 InertiaTensor2d<N>,
                                                 Transform2d<N>>;
pub type CollisionDetector2d<N> = DefaultDefaultCollisionDetector<Contact2d<N>, N,
                                                                  LinearVelocity2d<N>>;
pub type NarrowPhase2d<N>    = HasGeomHasGeomCollisionDetector<
                               Contact2d<N>,
                               RigidBody2d<N>,
                               RigidBody2d<N>,
                               Geom2d<N>,
                               Geom2d<N>,
                               CollisionDetector2d<N>>;
pub type BroadPhase2d<N>     = BruteForceBroadPhase<RigidBody2d<N>>;
pub type Contact2d<N>        = GeometricContactWithImpulse<LinearVelocity2d<N>, N>;
pub type ConstraintSolver2d<N> = AccumulatedImpulseSolver<
                                   N,
                                   Contact2d<N>,
                                   LinearVelocity2d<N>,
                                   AngularVelocity2d<N>,
                                   RigidBody2d<N>,
                                   InertiaTensor2d<N>,
                                   Transform2d<N>
                                 >;

pub type DefaultWorld2d<N> = World<RigidBody2d<N>,
                                   Integrator2d<N>,
                                   NarrowPhase2d<N>,
                                   BroadPhase2d<N>,
                                   ConstraintSolver2d<N>,
                                   Contact2d<N>,
                                   N>;

pub type Ball2d<N>        = Ball<N, Vec2<N>>;
pub type Plane2d<N>       = Plane<Vec2<N>>;
pub type DefaultGeom2d<N> = DefaultGeom<N, Vec2<N>>;

pub type IslandAccumulator2d<N> = IslandAccumulator<RigidBody2d<N>,
                                                    NarrowPhase2d<N>,
                                                    Contact2d<N>>;
