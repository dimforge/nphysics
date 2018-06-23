use kiss3d::scene::SceneNode2;
use kiss3d::window::Window;
use na::{Isometry2, Point3};
use nphysics2d::object::ColliderHandle;
use nphysics2d::world::World;
use objects::ball::Ball;
use objects::box_node::Box;
use objects::convex::Convex;
use objects::plane::Plane;
use objects::polyline::Polyline;

pub enum Node {
    Plane(Plane),
    Ball(Ball),
    Box(Box),
    Polyline(Polyline),
    Convex(Convex),
}

impl Node {
    pub fn select(&mut self) {
        match *self {
            Node::Plane(ref mut n) => n.select(),
            Node::Ball(ref mut n) => n.select(),
            Node::Box(ref mut n) => n.select(),
            Node::Polyline(ref mut n) => n.select(),
            Node::Convex(ref mut n) => n.select(),
        }
    }

    pub fn unselect(&mut self) {
        match *self {
            Node::Plane(ref mut n) => n.unselect(),
            Node::Ball(ref mut n) => n.unselect(),
            Node::Box(ref mut n) => n.unselect(),
            Node::Polyline(ref mut n) => n.unselect(),
            Node::Convex(ref mut n) => n.unselect(),
        }
    }

    pub fn update(&mut self, world: &World<f32>) {
        match *self {
            Node::Plane(ref mut n) => n.update(world),
            Node::Ball(ref mut n) => n.update(world),
            Node::Box(ref mut n) => n.update(world),
            Node::Polyline(ref mut n) => n.update(world),
            Node::Convex(ref mut n) => n.update(world),
        }
    }

    pub fn draw(&mut self, window: &mut Window) {
        match *self {
            Node::Polyline(ref mut n) => n.draw(window),
            Node::Plane(ref mut n) => n.draw(window),
            Node::Ball(..) => {}
            Node::Box(..) => {}
            Node::Convex(..) => {}
        }
    }

    pub fn scene_node(&self) -> Option<&SceneNode2> {
        match *self {
            Node::Plane(ref n) => None,
            Node::Ball(ref n) => Some(n.scene_node()),
            Node::Box(ref n) => Some(n.scene_node()),
            Node::Polyline(_) => None,
            Node::Convex(ref n) => Some(n.scene_node()),
        }
    }

    pub fn scene_node_mut(&mut self) -> Option<&mut SceneNode2> {
        match *self {
            Node::Plane(ref mut n) => None,
            Node::Ball(ref mut n) => Some(n.scene_node_mut()),
            Node::Box(ref mut n) => Some(n.scene_node_mut()),
            Node::Polyline(_) => None,
            Node::Convex(ref mut n) => Some(n.scene_node_mut()),
        }
    }

    pub fn collider(&self) -> ColliderHandle {
        match *self {
            Node::Plane(ref n) => n.object(),
            Node::Ball(ref n) => n.object(),
            Node::Box(ref n) => n.object(),
            Node::Polyline(ref n) => n.object(),
            Node::Convex(ref n) => n.object(),
        }
    }

    pub fn set_color(&mut self, color: Point3<f32>) {
        match *self {
            Node::Plane(ref mut n) => n.set_color(color),
            Node::Ball(ref mut n) => n.set_color(color),
            Node::Box(ref mut n) => n.set_color(color),
            Node::Polyline(ref mut n) => n.set_color(color),
            Node::Convex(ref mut n) => n.set_color(color),
        }
    }
}

pub fn update_scene_node(
    node: &mut SceneNode2,
    world: &World<f32>,
    coll: ColliderHandle,
    color: &Point3<f32>,
    delta: &Isometry2<f32>,
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
