use std::collections::HashMap;

use std::f32;
use na::{Isometry2, Vector2};
use ncollide::shape::{self, Shape};
use nphysics::object::{ColliderAnchor, Collider, DefaultBodySet, DefaultColliderSet, DefaultBodyHandle,
                       RigidBody};
use nphysics::joint::DefaultJointConstraintSet;
use nphysics::material::BasicMaterial;
use nphysics::force_generator::DefaultForceGeneratorSet;
use nphysics::world::DefaultDynamicWorld;

use wrapped2d::b2;
use wrapped2d::user_data::NoUserData;

fn na_vec_to_b2_vec(v: &Vector2<f32>) -> b2::Vec2 {
    b2::Vec2 { x: v.x, y: v.y }
}

fn b2_vec_to_na_vec(v: &b2::Vec2) -> Vector2<f32> {
    Vector2::new(v.x, v.y)
}

fn b2_transform_to_na_isometry(v: &b2::Transform) -> Isometry2<f32> {
    Isometry2::new(b2_vec_to_na_vec(&v.pos), v.rot.angle())
}

pub struct Box2dWorld {
    world: b2::World<NoUserData>,
    nphysics2box2d: HashMap<DefaultBodyHandle, b2::BodyHandle>,
}

impl Box2dWorld {
    pub fn from_nphysics(
        dynamic_world: &DefaultDynamicWorld<f32>,
        bodies: &DefaultBodySet<f32>,
        colliders: &DefaultColliderSet<f32>,
        _joint_constraints: &DefaultJointConstraintSet<f32>,
        _force_generators: &DefaultForceGeneratorSet<f32>
    )
    -> Self {
        let world = b2::World::new(&na_vec_to_b2_vec(&dynamic_world.gravity));

        let mut res = Box2dWorld {
            world,
            nphysics2box2d: HashMap::new(),
        };

        res.insert_bodies(bodies, colliders);
        res
    }

    fn insert_bodies(&mut self, bodies: &DefaultBodySet<f32>, colliders: &DefaultColliderSet<f32>) {
        for (handle, body) in bodies.iter() {
            let part = body.part(0).unwrap();
            let pos = part.position();
            let vel = part.velocity();

            let body_type = if body.is_static() {
                b2::BodyType::Static
            } else {
                b2::BodyType::Dynamic
            };

            let linear_damping;
            let angular_damping;

            if let Some(rb) = body.downcast_ref::<RigidBody<f32>>() {
                linear_damping = rb.linear_damping();
                angular_damping = rb.angular_damping();
            } else {
                linear_damping = 0.0;
                angular_damping = 0.0;
            }

            let def = b2::BodyDef {
                body_type,
                position: na_vec_to_b2_vec(&pos.translation.vector),
                angle: pos.rotation.angle(),
                linear_velocity: na_vec_to_b2_vec(&vel.linear),
                angular_velocity: vel.angular,
                linear_damping,
                angular_damping,
                .. b2::BodyDef::new()
            };
            let b2_handle = self.world.create_body(&def);
            self.nphysics2box2d.insert(handle, b2_handle);
        }


        for (_, collider) in colliders.iter() {
            match collider.anchor() {
                ColliderAnchor::OnBodyPart { body_part, .. } => {
                    if let Some(b2_handle) = self.nphysics2box2d.get(&body_part.0) {
                        let mut b2_body = self.world.body_mut(*b2_handle);

                        if collider.is_ccd_enabled() {
                            b2_body.set_bullet(true);
                        }

                        Self::create_fixtures(collider, collider.shape(), &collider.position_wrt_body(), &mut *b2_body);
                    }
                }
                ColliderAnchor::OnDeformableBody { .. } => println!("Deformable bodies are not supported by Box2D.")
            }
        }
    }

    fn create_fixtures(collider: &Collider<f32, DefaultBodyHandle>, shape: &Shape<f32>, dpos: &Isometry2<f32>, body: &mut b2::MetaBody<NoUserData>) {
        let center = na_vec_to_b2_vec(&dpos.translation.vector);
        let mut fixture_def = b2::FixtureDef::new();

        fixture_def.restitution = 0.0;
        fixture_def.friction = 0.5;
        fixture_def.density = collider.density();
        fixture_def.is_sensor = collider.is_sensor();
        fixture_def.filter = b2::Filter::new();

        if let Some(material) = collider.material().downcast_ref::<BasicMaterial<f32>>() {
            fixture_def.restitution = material.restitution;
            fixture_def.friction = material.friction;
        }

        if let Some(_s) = shape.as_shape::<shape::Plane<f32>>() {
        } else if let Some(s) = shape.as_shape::<shape::Ball<f32>>() {
            let mut b2_shape = b2::CircleShape::new();
            b2_shape.set_radius(s.radius());
            b2_shape.set_position(center);
            body.create_fixture(&b2_shape, &mut fixture_def);
        } else if let Some(s) = shape.as_shape::<shape::Cuboid<f32>>() {
            let half_extents = s.half_extents();
            let b2_shape = b2::PolygonShape::new_oriented_box(half_extents.x, half_extents.y, &center, dpos.rotation.angle());
            body.create_fixture(&b2_shape, &mut fixture_def);
        } else if let Some(_s) = shape.as_shape::<shape::Capsule<f32>>() {
//            let geom = PhysicsGeometry::from(&ColliderDesc::Capsule(s.radius(), s.height()));
//            result.push((geom, Isometry3::rotation(Vector3::z() * f32::consts::PI / 2.0)))
        } else if let Some(cp) = shape.as_shape::<shape::Compound<f32>>() {
            for (shift, shape) in cp.shapes() {
                Self::create_fixtures(collider, &**shape, &(dpos * shift), body)
            }
        } else if let Some(ch) = shape.as_shape::<shape::ConvexPolygon<f32>>() {
            let points: Vec<_> = ch.points()
                .iter()
                .map(|p| dpos * p)
                .map(|p| na_vec_to_b2_vec(&p.coords))
                .collect();
            let b2_shape = b2::PolygonShape::new_with(&points);
            body.create_fixture(&b2_shape, &mut fixture_def);
        } else if let Some(_s) = shape.as_shape::<shape::Polyline<f32>>() {
        } else if let Some(_s) = shape.as_shape::<shape::HeightField<f32>>() {
        }
    }

    pub fn step(&mut self, dynamic_world: &mut DefaultDynamicWorld<f32>) {
        self.world.set_continuous_physics(dynamic_world.integration_parameters.max_ccd_substeps != 0);

        dynamic_world.counters.step_started();
        self.world.step(
            dynamic_world.integration_parameters.dt(),
            dynamic_world.integration_parameters.max_velocity_iterations as i32,
            dynamic_world.integration_parameters.max_position_iterations as i32);
        dynamic_world.counters.step_completed();
    }

    pub fn sync(&self,
                _bodies: &mut DefaultBodySet<f32>,
                colliders: &mut DefaultColliderSet<f32>) {
        for (_, collider) in colliders.iter_mut() {
            match collider.anchor() {
                ColliderAnchor::OnBodyPart { body_part, position_wrt_body_part } => {
                    if let Some(pb2_handle) = self.nphysics2box2d.get(&body_part.0) {
                        let body = self.world.body(*pb2_handle);
                        let pos = b2_transform_to_na_isometry(body.transform());
                        let new_pos = pos * position_wrt_body_part;
                        collider.set_position(new_pos)
                    }
                }
                ColliderAnchor::OnDeformableBody { .. } => {}
            }
        }
    }
}
