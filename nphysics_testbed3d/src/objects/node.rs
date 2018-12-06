use kiss3d::scene::SceneNode;
use na::{Isometry3, Point3};
use nphysics3d::object::ColliderHandle;
use nphysics3d::world::World;
use crate::objects::ball::Ball;
use crate::objects::box_node::Box;
use crate::objects::cone::Cone;
use crate::objects::convex::Convex;
use crate::objects::cylinder::Cylinder;
use crate::objects::mesh::Mesh;
use crate::objects::plane::Plane;

pub enum Node {
    Plane(Plane),
    Ball(Ball),
    Box(Box),
    Cylinder(Cylinder),
    Cone(Cone),
    Mesh(Mesh),
    Convex(Convex),
}

impl Node {
    pub fn select(&mut self) {
        match *self {
            Node::Plane(ref mut n) => n.select(),
            Node::Ball(ref mut n) => n.select(),
            Node::Box(ref mut n) => n.select(),
            Node::Cylinder(ref mut n) => n.select(),
            Node::Cone(ref mut n) => n.select(),
            Node::Mesh(ref mut n) => n.select(),
            Node::Convex(ref mut n) => n.select(),
        }
    }

    pub fn unselect(&mut self) {
        match *self {
            Node::Plane(ref mut n) => n.unselect(),
            Node::Ball(ref mut n) => n.unselect(),
            Node::Box(ref mut n) => n.unselect(),
            Node::Cylinder(ref mut n) => n.unselect(),
            Node::Cone(ref mut n) => n.unselect(),
            Node::Mesh(ref mut n) => n.unselect(),
            Node::Convex(ref mut n) => n.unselect(),
        }
    }

    pub fn update(&mut self, world: &World<f32>) {
        match *self {
            Node::Plane(ref mut n) => n.update(world),
            Node::Ball(ref mut n) => n.update(world),
            Node::Box(ref mut n) => n.update(world),
            Node::Cylinder(ref mut n) => n.update(world),
            Node::Cone(ref mut n) => n.update(world),
            Node::Mesh(ref mut n) => n.update(world),
            Node::Convex(ref mut n) => n.update(world),
        }
    }

    pub fn scene_node(&self) -> &SceneNode {
        match *self {
            Node::Plane(ref n) => n.scene_node(),
            Node::Ball(ref n) => n.scene_node(),
            Node::Box(ref n) => n.scene_node(),
            Node::Cylinder(ref n) => n.scene_node(),
            Node::Cone(ref n) => n.scene_node(),
            Node::Mesh(ref n) => n.scene_node(),
            Node::Convex(ref n) => n.scene_node(),
        }
    }

    pub fn scene_node_mut(&mut self) -> &mut SceneNode {
        match *self {
            Node::Plane(ref mut n) => n.scene_node_mut(),
            Node::Ball(ref mut n) => n.scene_node_mut(),
            Node::Box(ref mut n) => n.scene_node_mut(),
            Node::Cylinder(ref mut n) => n.scene_node_mut(),
            Node::Cone(ref mut n) => n.scene_node_mut(),
            Node::Mesh(ref mut n) => n.scene_node_mut(),
            Node::Convex(ref mut n) => n.scene_node_mut(),
        }
    }

    pub fn collider(&self) -> ColliderHandle {
        match *self {
            Node::Plane(ref n) => n.object(),
            Node::Ball(ref n) => n.object(),
            Node::Box(ref n) => n.object(),
            Node::Cylinder(ref n) => n.object(),
            Node::Cone(ref n) => n.object(),
            Node::Mesh(ref n) => n.object(),
            Node::Convex(ref n) => n.object(),
        }
    }

    pub fn set_color(&mut self, color: Point3<f32>) {
        match *self {
            Node::Plane(ref mut n) => n.set_color(color),
            Node::Ball(ref mut n) => n.set_color(color),
            Node::Box(ref mut n) => n.set_color(color),
            Node::Cylinder(ref mut n) => n.set_color(color),
            Node::Cone(ref mut n) => n.set_color(color),
            Node::Mesh(ref mut n) => n.set_color(color),
            Node::Convex(ref mut n) => n.set_color(color),
        }
    }
}

pub fn update_scene_node(
    node: &mut SceneNode,
    world: &World<f32>,
    coll: ColliderHandle,
    color: &Point3<f32>,
    delta: &Isometry3<f32>,
) {
    let co = world.collider(coll).unwrap();
    // let active = world.body(co.data().body()).is_active();

    if true {
        // active {
        node.set_local_transformation(co.position() * delta);
        node.set_color(color.x, color.y, color.z);
    } else {
        node.set_color(color.x * 0.25, color.y * 0.25, color.z * 0.25);
    }
}
