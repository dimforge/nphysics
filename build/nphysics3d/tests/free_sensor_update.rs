extern crate nalgebra as na;
extern crate ncollide;
extern crate nphysics3d;

use ncollide::shape::{Ball};
use nphysics3d::world::World;
use nphysics3d::object::{RigidBody, Sensor};

#[test]
fn free_sensor_update() {
    let mut world = World::new();

    let mut ball = RigidBody::new_dynamic(Ball::new(0.5), 1.0, 0.3, 0.5);
    ball.set_transformation(na::Isometry3::from_parts(na::Translation3::new(1.0, 1.0, 1.0), na::one()));
    world.add_rigid_body(ball);

    let mut sensor = Sensor::new(Ball::new(0.1), None);
    sensor.enable_interfering_bodies_collection();
    sensor.set_relative_position(na::Isometry3::from_parts(na::Translation3::new(0.0, 0.0, 0.0), na::one()));

    let sensor = world.add_sensor(sensor);
    world.step(1.0);
    assert_eq!(world.sensor(sensor).interfering_bodies().unwrap().len(), 0);

    world.mut_sensor(sensor).set_relative_position(na::Isometry3::from_parts(na::Translation3::new(1.0, 1.0, 1.0), na::one()));
    world.step(1.0);
    assert_eq!(world.sensor(sensor).interfering_bodies().unwrap().len(), 1);
}
