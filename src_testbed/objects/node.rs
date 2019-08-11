use na::Point3;
use kiss3d::window::Window;
use nphysics::object::{DefaultColliderHandle, DefaultColliderSet};
use nphysics::math::Isometry;
use crate::objects::ball::Ball;
use crate::objects::box_node::Box;
use crate::objects::capsule::Capsule;
use crate::objects::convex::Convex;
#[cfg(feature = "dim2")]
use crate::objects::polyline::Polyline;
#[cfg(feature = "dim3")]
use crate::objects::mesh::Mesh;
use crate::objects::heightfield::HeightField;
use crate::objects::plane::Plane;
use crate::objects::multiball::Multiball;

#[cfg(feature = "dim2")]
pub type GraphicsNode = kiss3d::scene::PlanarSceneNode;
#[cfg(feature = "dim3")]
pub type GraphicsNode = kiss3d::scene::SceneNode;

pub enum Node {
    Plane(Plane),
    Ball(Ball),
    Multiball(Multiball),
    Box(Box),
    HeightField(HeightField),
    Capsule(Capsule),
    #[cfg(feature = "dim2")]
    Polyline(Polyline),
    #[cfg(feature = "dim3")]
    Mesh(Mesh),
    Convex(Convex),
}

impl Node {
    pub fn select(&mut self) {
        match *self {
            Node::Plane(ref mut n) => n.select(),
            Node::Ball(ref mut n) => n.select(),
            Node::Multiball(ref mut n) => n.select(),
            Node::Box(ref mut n) => n.select(),
            Node::Capsule(ref mut n) => n.select(),
            Node::HeightField(ref mut n) => n.select(),
            #[cfg(feature = "dim2")]
            Node::Polyline(ref mut n) => n.select(),
            #[cfg(feature = "dim3")]
            Node::Mesh(ref mut n) => n.select(),
            Node::Convex(ref mut n) => n.select(),
        }
    }

    pub fn unselect(&mut self) {
        match *self {
            Node::Plane(ref mut n) => n.unselect(),
            Node::Ball(ref mut n) => n.unselect(),
            Node::Multiball(ref mut n) => n.unselect(),
            Node::Box(ref mut n) => n.unselect(),
            Node::Capsule(ref mut n) => n.unselect(),
            Node::HeightField(ref mut n) => n.unselect(),
            #[cfg(feature = "dim2")]
            Node::Polyline(ref mut n) => n.unselect(),
            #[cfg(feature = "dim3")]
            Node::Mesh(ref mut n) => n.unselect(),
            Node::Convex(ref mut n) => n.unselect(),
        }
    }

    pub fn update(&mut self, colliders: &DefaultColliderSet<f32>) {
        match *self {
            Node::Plane(ref mut n) => n.update(colliders),
            Node::Ball(ref mut n) => n.update(colliders),
            Node::Multiball(ref mut n) => n.update(colliders),
            Node::Box(ref mut n) => n.update(colliders),
            Node::Capsule(ref mut n) => n.update(colliders),
            Node::HeightField(ref mut n) => n.update(colliders),
            #[cfg(feature = "dim2")]
            Node::Polyline(ref mut n) => n.update(colliders),
            #[cfg(feature = "dim3")]
            Node::Mesh(ref mut n) => n.update(colliders),
            Node::Convex(ref mut n) => n.update(colliders),
        }
    }

    pub fn draw(&mut self, _window: &mut Window) {
        match *self {
            #[cfg(feature = "dim2")]
            Node::Polyline(ref mut n) => n.draw(_window),
            #[cfg(feature = "dim2")]
            Node::HeightField(ref mut n) => n.draw(_window),
            #[cfg(feature = "dim2")]
            Node::Plane(ref mut n) => n.draw(_window),
            _ => {}
        }
    }

    pub fn scene_node(&self) -> Option<&GraphicsNode> {
        match *self {
            #[cfg(feature = "dim3")]
            Node::Plane(ref n) => Some(n.scene_node()),
            Node::Ball(ref n) => Some(n.scene_node()),
            Node::Multiball(ref n) => Some(n.scene_node()),
            Node::Box(ref n) => Some(n.scene_node()),
            Node::Capsule(ref n) => Some(n.scene_node()),
            #[cfg(feature = "dim3")]
            Node::HeightField(ref n) => Some(n.scene_node()),
            #[cfg(feature = "dim3")]
            Node::Mesh(ref n) => Some(n.scene_node()),
            Node::Convex(ref n) => Some(n.scene_node()),
            #[cfg(feature = "dim2")]
            _ => None,
        }
    }

    pub fn scene_node_mut(&mut self) -> Option<&mut GraphicsNode> {
        match *self {
            #[cfg(feature = "dim3")]
            Node::Plane(ref mut n) => Some(n.scene_node_mut()),
            Node::Ball(ref mut n) => Some(n.scene_node_mut()),
            Node::Multiball(ref mut n) => Some(n.scene_node_mut()),
            Node::Box(ref mut n) => Some(n.scene_node_mut()),
            Node::Capsule(ref mut n) => Some(n.scene_node_mut()),
            #[cfg(feature = "dim3")]
            Node::HeightField(ref mut n) => Some(n.scene_node_mut()),
            #[cfg(feature = "dim3")]
            Node::Mesh(ref mut n) => Some(n.scene_node_mut()),
            Node::Convex(ref mut n) => Some(n.scene_node_mut()),
            #[cfg(feature = "dim2")]
            _ => None,
        }
    }

    pub fn collider(&self) -> DefaultColliderHandle {
        match *self {
            Node::Plane(ref n) => n.object(),
            Node::Ball(ref n) => n.object(),
            Node::Multiball(ref n) => n.object(),
            Node::Box(ref n) => n.object(),
            Node::Capsule(ref n) => n.object(),
            Node::HeightField(ref n) => n.object(),
            #[cfg(feature = "dim2")]
            Node::Polyline(ref n) => n.object(),
            #[cfg(feature = "dim3")]
            Node::Mesh(ref n) => n.object(),
            Node::Convex(ref n) => n.object(),
        }
    }

    pub fn set_color(&mut self, color: Point3<f32>) {
        match *self {
            Node::Plane(ref mut n) => n.set_color(color),
            Node::Ball(ref mut n) => n.set_color(color),
            Node::Multiball(ref mut n) => n.set_color(color),
            Node::Box(ref mut n) => n.set_color(color),
            Node::Capsule(ref mut n) => n.set_color(color),
            Node::HeightField(ref mut n) => n.set_color(color),
            #[cfg(feature = "dim2")]
            Node::Polyline(ref mut n) => n.set_color(color),
            #[cfg(feature = "dim3")]
            Node::Mesh(ref mut n) => n.set_color(color),
            Node::Convex(ref mut n) => n.set_color(color),
        }
    }
}

pub fn update_scene_node(
    node: &mut GraphicsNode,
    colliders: &DefaultColliderSet<f32>,
    coll: DefaultColliderHandle,
    color: &Point3<f32>,
    delta: &Isometry<f32>,
) {
    if let Some(co) = colliders.get(coll) {
        node.set_local_transformation(co.position() * delta);
        node.set_color(color.x, color.y, color.z);
    } else {
        node.set_color(color.x * 0.25, color.y * 0.25, color.z * 0.25);
    }
}
