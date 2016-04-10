use std::f32;
use sfml::graphics::{Color, Shape, Transformable};
use sfml::system::Vector2f;
use na::{self, Pnt3, Iso2};
use nphysics2d::object::{WorldObject, WorldObjectBorrowed};
use objects::{Ball, Box, Lines, Segment};
use draw_helper::DRAW_SCALE;

pub enum SceneNode<'a> {
    BallNode(Ball<'a>),
    BoxNode(Box<'a>),
    LinesNode(Lines),
    SegmentNode(Segment)
}

impl<'a> SceneNode<'a> {
    pub fn select(&mut self) {
        match *self {
            SceneNode::BallNode(ref mut n)    => n.select(),
            SceneNode::BoxNode(ref mut n)     => n.select(),
            SceneNode::LinesNode(ref mut n)   => n.select(),
            SceneNode::SegmentNode(ref mut n) => n.select(),
        }
    }

    pub fn unselect(&mut self) {
        match *self {
            SceneNode::BallNode(ref mut n)    => n.unselect(),
            SceneNode::BoxNode(ref mut n)     => n.unselect(),
            SceneNode::LinesNode(ref mut n)   => n.unselect(),
            SceneNode::SegmentNode(ref mut n) => n.unselect(),
        }
    }

    pub fn set_color(&mut self, color: Pnt3<u8>) {
        match *self {
            SceneNode::BallNode(ref mut n)    => n.set_color(color),
            SceneNode::BoxNode(ref mut n)     => n.set_color(color),
            SceneNode::LinesNode(ref mut n)   => n.set_color(color),
            SceneNode::SegmentNode(ref mut n) => n.set_color(color),
        }
    }
}

pub fn update_scene_node<'a, T>(node:   &mut T,
                                object: &WorldObject<f32>,
                                color:  &Pnt3<u8>,
                                delta:  &Iso2<f32>)
        where T: Transformable + Shape<'a> {
    let bobject   = object.borrow();
    let transform = bobject.position() * *delta;
    let pos       = na::translation(&transform);
    let rot       = na::rotation(&transform);

    node.set_position(&Vector2f::new(pos.x as f32 * DRAW_SCALE, pos.y as f32 * DRAW_SCALE));
    node.set_rotation(rot.x * 180.0 / f32::consts::PI as f32);

    match bobject {
        WorldObjectBorrowed::Sensor(_) => {
            // FIXME: what is the impact on performance of doing those at each update.
            node.set_fill_color(&Color::new_rgba(0, 0, 0, 0));
            node.set_outline_thickness(2.0);
            node.set_outline_color(&Color::new_rgb(color.x, color.y, color.z));
        },
        WorldObjectBorrowed::RigidBody(rb) => {
            if !rb.is_active() {
                node.set_fill_color(&Color::new_rgb(color.x / 4, color.y / 4, color.z / 4));
            }
            else {
                node.set_fill_color(&Color::new_rgb(color.x, color.y, color.z));
            }
        }
    }
}
