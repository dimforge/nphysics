use nalgebra::dim3::mat3::Mat3;
use nalgebra::dim3::vec3::Vec3;
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

pub type Transform3d<N>       =  Transform<Rotmat<Mat3<N>>, Vec3<N>>;
pub type LinearVelocity3d<N>  = Vec3<N>;
pub type AngularVelocity3d<N> = Vec3<N>;
pub type InertiaTensor3d<N>   = Mat3<N>;
pub type Geom3d<N>            = DefaultGeom<N, LinearVelocity3d<N>>;
pub type RigidBody3d<N>       = RigidBody<Geom3d<N>,
                                          N,
                                          Transform3d<N>,
                                          LinearVelocity3d<N>,
                                          AngularVelocity3d<N>,
                                          InertiaTensor3d<N>>;

pub type Integrator3d<N> = BodyGravityIntegrator<LinearVelocity3d<N>,
                                                 AngularVelocity3d<N>,
                                                 N,
                                                 RigidBody3d<N>,
                                                 InertiaTensor3d<N>,
                                                 Transform3d<N>>;
pub type CollisionDetector3d<N> = DefaultDefaultCollisionDetector<Contact3d<N>, N,
                                                                  LinearVelocity3d<N>>;
pub type NarrowPhase3d<N>    = HasGeomHasGeomCollisionDetector<
                               Contact3d<N>,
                               RigidBody3d<N>,
                               RigidBody3d<N>,
                               Geom3d<N>,
                               Geom3d<N>,
                               CollisionDetector3d<N>>;
pub type BroadPhase3d<N>     = BruteForceBroadPhase<RigidBody3d<N>>;
pub type Contact3d<N>        = GeometricContactWithImpulse<LinearVelocity3d<N>, N>;
pub type ConstraintSolver3d<N> = AccumulatedImpulseSolver<
                                   N,
                                   Contact3d<N>,
                                   LinearVelocity3d<N>,
                                   AngularVelocity3d<N>,
                                   RigidBody3d<N>,
                                   InertiaTensor3d<N>,
                                   Transform3d<N>
                                 >;

pub type DefaultWorld3d<N> = World<RigidBody3d<N>,
                                   Integrator3d<N>,
                                   NarrowPhase3d<N>,
                                   BroadPhase3d<N>,
                                   ConstraintSolver3d<N>,
                                   Contact3d<N>,
                                   N>;

pub type Ball3d<N>        = Ball<N, Vec3<N>>;
pub type Plane3d<N>       = Plane<Vec3<N>>;
pub type DefaultGeom3d<N> = DefaultGeom<N, Vec3<N>>;

pub type IslandAccumulator3d<N> = IslandAccumulator<RigidBody3d<N>,
                                                    NarrowPhase3d<N>,
                                                    Contact3d<N>>;
