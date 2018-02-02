use std::f32;
use sfml::graphics::{Color, Shape, Transformable};
use sfml::system::Vector2f;
use na::{Isometry2, Point3};
use nphysics2d::world::World;
use nphysics2d::object::ColliderHandle;
use objects::{Ball, Box, Lines, Segment};
use draw_helper::DRAW_SCALE;

pub enum SceneNode<'a> {
    BallNode(Ball<'a>),
    BoxNode(Box<'a>),
    LinesNode(Lines),
    SegmentNode(Segment),
}

impl<'a> SceneNode<'a> {
    pub fn collider(&self) -> ColliderHandle {
        match *self {
            SceneNode::BallNode(ref n) => n.collider(),
            SceneNode::BoxNode(ref n) => n.collider(),
            SceneNode::LinesNode(ref n) => n.collider(),
            SceneNode::SegmentNode(ref n) => n.collider(),
        }
    }

    pub fn select(&mut self) {
        match *self {
            SceneNode::BallNode(ref mut n) => n.select(),
            SceneNode::BoxNode(ref mut n) => n.select(),
            SceneNode::LinesNode(ref mut n) => n.select(),
            SceneNode::SegmentNode(ref mut n) => n.select(),
        }
    }

    pub fn unselect(&mut self) {
        match *self {
            SceneNode::BallNode(ref mut n) => n.unselect(),
            SceneNode::BoxNode(ref mut n) => n.unselect(),
            SceneNode::LinesNode(ref mut n) => n.unselect(),
            SceneNode::SegmentNode(ref mut n) => n.unselect(),
        }
    }

    pub fn set_color(&mut self, color: Point3<u8>) {
        match *self {
            SceneNode::BallNode(ref mut n) => n.set_color(color),
            SceneNode::BoxNode(ref mut n) => n.set_color(color),
            SceneNode::LinesNode(ref mut n) => n.set_color(color),
            SceneNode::SegmentNode(ref mut n) => n.set_color(color),
        }
    }
}

pub fn update_scene_node<'a, T>(
    node: &mut T,
    id: ColliderHandle,
    world: &World<f32>,
    color: &Point3<u8>,
    delta: &Isometry2<f32>,
) where
    T: Transformable + Shape<'a>,
{
    let collider = world.collider(id).unwrap();
    let active = world.body(collider.data().body()).is_active();

    let iso = collider.position() * delta;
    let pos = iso.translation.vector;
    let rot = iso.rotation.angle();

    node.set_position(&Vector2f::new(
        pos.x as f32 * DRAW_SCALE,
        pos.y as f32 * DRAW_SCALE,
    ));
    node.set_rotation(rot * 180.0 / f32::consts::PI as f32);

    let color = if !active {
        Color::new_rgb(color.x / 4, color.y / 4, color.z / 4)
    } else {
        Color::new_rgb(color.x, color.y, color.z)
    };

    if collider.query_type().is_proximity_query() {
        node.set_fill_color(&Color::new_rgba(0, 0, 0, 0));
        node.set_outline_thickness(1.0);
        node.set_outline_color(&color);
    } else {
        node.set_fill_color(&color);
    }
}
