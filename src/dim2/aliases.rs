use nalgebra::dim2::mat2::Mat2;
use nalgebra::dim2::vec2::Vec2;
use nalgebra::dim1::vec1::Vec1;
use nalgebra::adaptors::rotmat::Rotmat;
use nalgebra::adaptors::transform::Transform;
use body::rigid_body::RigidBody;
use integrator::body_gravity_integrator::BodyGravityIntegrator;
use world::world::World;

type T                 = f64; // FIXME: really put this here?

type Transform2d       = Transform<Rotmat<Mat2<T>>, Vec2<T>>;
type Shape2d           = T;           // FIXME
type LinearVelocity2d  = Vec2<T>;
type AngularVelocity2d = Vec1<T>;
type InertiaTensor2d   = T;
type RigidBody2d       = RigidBody<Shape2d,
                                   T,
                                   Transform2d,
                                   LinearVelocity2d,
                                   AngularVelocity2d,
                                   InertiaTensor2d>;

type Integrator2d = BodyGravityIntegrator<LinearVelocity2d,
                                          AngularVelocity2d,
                                          T,
                                          RigidBody2d,
                                          InertiaTensor2d,
                                          Transform2d>;
type NarrowPhase2d       = T; // FIXME
type BroadPhase2d        = T; // FIXME
type CollisionDetector2d = T; // FIXME
type CollisionSolver2d   = T; // FIXME

type DefaultWorld2d = World<RigidBody2d,
                            Integrator2d,
                            NarrowPhase2d,
                            BroadPhase2d,
                            CollisionDetector2d,
                            CollisionSolver2d,
                            T>;
