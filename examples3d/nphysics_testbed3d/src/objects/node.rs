use kiss3d::scene::SceneNode;
use na::{Iso3, Pnt3};
use nphysics3d::object::{WorldObject, WorldObjectBorrowed};
use objects::ball::Ball;
use objects::box_node::Box;
use objects::cylinder::Cylinder;
use objects::cone::Cone;
use objects::mesh::Mesh;
use objects::plane::Plane;
use objects::convex::Convex;

pub enum Node {
    Ball(Ball),
    Box(Box),
    Cylinder(Cylinder),
    Cone(Cone),
    Mesh(Mesh),
    Plane(Plane),
    Convex(Convex)
}

impl Node {
    pub fn select(&mut self) {
        match *self {
            Node::Plane(ref mut n)    => n.select(),
            Node::Ball(ref mut n)     => n.select(),
            Node::Box(ref mut n)      => n.select(),
            Node::Cylinder(ref mut n) => n.select(),
            Node::Cone(ref mut n)     => n.select(),
            Node::Mesh(ref mut n)     => n.select(),
            Node::Convex(ref mut n)   => n.select()
        }
    }

    pub fn unselect(&mut self) {
        match *self {
            Node::Plane(ref mut n)    => n.unselect(),
            Node::Ball(ref mut n)     => n.unselect(),
            Node::Box(ref mut n)      => n.unselect(),
            Node::Cylinder(ref mut n) => n.unselect(),
            Node::Cone(ref mut n)     => n.unselect(),
            Node::Mesh(ref mut n)     => n.unselect(),
            Node::Convex(ref mut n)   => n.unselect()
        }
    }

    pub fn update(&mut self) {
        match *self {
            Node::Plane(ref mut n)    => n.update(),
            Node::Ball(ref mut n)     => n.update(),
            Node::Box(ref mut n)      => n.update(),
            Node::Cylinder(ref mut n) => n.update(),
            Node::Cone(ref mut n)     => n.update(),
            Node::Mesh(ref mut n)     => n.update(),
            Node::Convex(ref mut n)   => n.update()
        }
    }

    pub fn scene_node(&self) -> &SceneNode {
        match *self {
            Node::Plane(ref n)    => n.scene_node(),
            Node::Ball(ref n)     => n.scene_node(),
            Node::Box(ref n)      => n.scene_node(),
            Node::Cylinder(ref n) => n.scene_node(),
            Node::Cone(ref n)     => n.scene_node(),
            Node::Mesh(ref n)     => n.scene_node(),
            Node::Convex(ref n)   => n.scene_node()
        }
    }

    pub fn scene_node_mut(&mut self) -> &mut SceneNode {
        match *self {
            Node::Plane(ref mut n)    => n.scene_node_mut(),
            Node::Ball(ref mut n)     => n.scene_node_mut(),
            Node::Box(ref mut n)      => n.scene_node_mut(),
            Node::Cylinder(ref mut n) => n.scene_node_mut(),
            Node::Cone(ref mut n)     => n.scene_node_mut(),
            Node::Mesh(ref mut n)     => n.scene_node_mut(),
            Node::Convex(ref mut n)   => n.scene_node_mut()
        }
    }

    pub fn object<'a>(&'a self) -> &'a WorldObject<f32> {
        match *self {
            Node::Plane(ref n)    => n.object(),
            Node::Ball(ref n)     => n.object(),
            Node::Box(ref n)      => n.object(),
            Node::Cylinder(ref n) => n.object(),
            Node::Cone(ref n)     => n.object(),
            Node::Mesh(ref n)     => n.object(),
            Node::Convex(ref n)   => n.object()
        }
    }

    pub fn set_color(&mut self, color: Pnt3<f32>)  {
        match *self {
            Node::Plane(ref mut n)    => n.set_color(color),
            Node::Ball(ref mut n)     => n.set_color(color),
            Node::Box(ref mut n)      => n.set_color(color),
            Node::Cylinder(ref mut n) => n.set_color(color),
            Node::Cone(ref mut n)     => n.set_color(color),
            Node::Mesh(ref mut n)     => n.set_color(color),
            Node::Convex(ref mut n)   => n.set_color(color)
        }
    }
}


pub fn update_scene_node(node:   &mut SceneNode,
                         object: &WorldObject<f32>,
                         color:  &Pnt3<f32>,
                         delta:  &Iso3<f32>) {
    let rb = object.borrow();

    match rb {
        WorldObjectBorrowed::RigidBody(rb) => {
            if rb.is_active() {
                node.set_local_transformation(*rb.position() * *delta);
                node.set_color(color.x, color.y, color.z);
            }
            else {
                node.set_color(color.x * 0.25, color.y * 0.25, color.z * 0.25);
            }
        },
        WorldObjectBorrowed::Sensor(s) => {
            if let Some(rb) = s.parent() {
                if rb.borrow().is_active() {
                    node.set_local_transformation(s.position() * *delta);
                }
            }
        }
    }
}
