use kiss3d::planar_camera::{PlanarCamera, Sidescroll};
use kiss3d::scene::PlanarSceneNode;
use kiss3d::window::Window;
use na;
use na::{Isometry2, Point2, Point3};
use ncollide2d::shape::{self, Compound, ConvexPolygon, Cuboid, Shape};
use nphysics2d::object::{Body, BodyHandle, ColliderHandle};
use nphysics2d::world::World;
use objects::ball::Ball;
use objects::box_node::Box;
use objects::convex::Convex;
use objects::polyline::Polyline;
// use objects::mesh::Mesh;
use objects::node::Node;
use objects::plane::Plane;
use rand::{Rng, SeedableRng, XorShiftRng};
use std::collections::HashMap;

pub struct GraphicsManager {
    rand: XorShiftRng,
    b2sn: HashMap<BodyHandle, Vec<Node>>,
    b2color: HashMap<BodyHandle, Point3<f32>>,
    c2color: HashMap<ColliderHandle, Point3<f32>>,
    camera: Sidescroll,
    aabbs: Vec<PlanarSceneNode>,
}

impl GraphicsManager {
    pub fn new() -> GraphicsManager {
        let rng: XorShiftRng = SeedableRng::from_seed([0; 16]);

        let mut camera = Sidescroll::new();
        camera.set_zoom(50.0);

        GraphicsManager {
            camera,
            rand: rng,
            b2sn: HashMap::new(),
            b2color: HashMap::new(),
            c2color: HashMap::new(),
            aabbs: Vec::new(),
        }
    }

    pub fn camera(&self) -> &Sidescroll {
        &self.camera
    }

    pub fn clear(&mut self, window: &mut Window) {
        for sns in self.b2sn.values() {
            for sn in sns.iter() {
                if let Some(node) = sn.scene_node() {
                    window.remove_planar_node(&mut node.clone());
                }
            }
        }

        for aabb in self.aabbs.iter_mut() {
            window.remove_planar_node(aabb);
        }

        self.b2sn.clear();
        self.aabbs.clear();
    }

    pub fn remove_body_nodes(&mut self, world: &World<f32>, window: &mut Window, body: BodyHandle) {
        let body_key = Self::body_key(world, body);

        if let Some(sns) = self.b2sn.get(&body_key) {
            for sn in sns.iter() {
                if let Some(node) = sn.scene_node() {
                    window.remove_planar_node(&mut node.clone());
                }
            }
        }

        self.b2sn.remove(&body_key);
    }

    pub fn remove_body_part_nodes(
        &mut self,
        world: &World<f32>,
        window: &mut Window,
        body: BodyHandle,
    ) -> BodyHandle {
        let mut delete_array = true;
        let body_key = Self::body_key(world, body);

        if let Some(sns) = self.b2sn.get_mut(&body_key) {
            sns.retain(|sn| {
                if world.collider(sn.collider()).unwrap().data().body() == body {
                    if let Some(node) = sn.scene_node() {
                        window.remove_planar_node(&mut node.clone());
                    }
                    false
                } else {
                    delete_array = false;
                    true
                }
            });
        }

        if delete_array {
            self.b2sn.remove(&body_key);
        }

        body_key
    }

    pub fn update_after_body_key_change(&mut self, world: &World<f32>, body_key: BodyHandle) {
        if let Some(color) = self.b2color.remove(&body_key) {
            if let Some(sns) = self.b2sn.remove(&body_key) {
                for sn in sns {
                    let sn_body = world.collider(sn.collider()).unwrap().data().body();
                    let sn_key = Self::body_key(world, sn_body);

                    let _ = self.b2color.entry(sn_key).or_insert(color);
                    let new_sns = self.b2sn.entry(sn_key).or_insert_with(Vec::new);
                    new_sns.push(sn);
                }
            }
        }
    }

    pub fn set_body_color(&mut self, world: &World<f32>, b: BodyHandle, color: Point3<f32>) {
        let body_key = Self::body_key(world, b);
        self.b2color.insert(body_key, color);

        if let Some(ns) = self.b2sn.get_mut(&body_key) {
            for n in ns.iter_mut() {
                n.set_color(color)
            }
        }
    }

    pub fn set_collider_color(&mut self, handle: ColliderHandle, color: Point3<f32>) {
        self.c2color.insert(handle, color);
    }

    fn body_key(world: &World<f32>, handle: BodyHandle) -> BodyHandle {
        if let Body::Multibody(mb) = world.body(handle) {
            mb.handle()
        } else {
            handle
        }
    }

    pub fn add(&mut self, window: &mut Window, id: ColliderHandle, world: &World<f32>) {
        let mut color = Point3::new(0.5, 0.5, 0.5);
        let collider = world.collider(id).unwrap();

        if let Some(c) = self.c2color.get(&id).cloned() {
            color = c
        } else {
            let body_key = Self::body_key(world, collider.data().body());
            match self.b2color.get(&body_key) {
                Some(c) => color = *c,
                None => {
                    if !collider.data().body().is_ground() {
                        color = self.rand.gen();
                        color *= 1.5;
                        color.x = color.x.min(1.0);
                        color.y = color.y.min(1.0);
                        color.z = color.z.min(1.0);
                    }
                }
            }

            self.set_body_color(world, collider.data().body(), color);
        }

        self.add_with_color(window, id, world, color)
    }

    pub fn add_with_color(
        &mut self,
        window: &mut Window,
        id: ColliderHandle,
        world: &World<f32>,
        color: Point3<f32>,
    ) {
        let collider = world.collider(id).unwrap();
        let parent = collider.data().body();
        let shape = collider.shape().as_ref();

        // NOTE: not optimal allocation-wise, but it is not critical here.
        let mut new_nodes = Vec::new();
        self.add_shape(window, id, world, na::one(), shape, color, &mut new_nodes);

        {
            let key = Self::body_key(world, parent);
            let nodes = self.b2sn.entry(key).or_insert_with(Vec::new);
            nodes.append(&mut new_nodes);
        }
    }

    fn add_shape(
        &mut self,
        window: &mut Window,
        object: ColliderHandle,
        world: &World<f32>,
        delta: Isometry2<f32>,
        shape: &Shape<f32>,
        color: Point3<f32>,
        out: &mut Vec<Node>,
    ) {
        if let Some(s) = shape.as_shape::<shape::Plane<f32>>() {
            self.add_plane(window, object, world, s, color, out)
        } else if let Some(s) = shape.as_shape::<shape::Ball<f32>>() {
            self.add_ball(window, object, world, delta, s, color, out)
        } else if let Some(s) = shape.as_shape::<Cuboid<f32>>() {
            self.add_box(window, object, world, delta, s, color, out)
        } else if let Some(s) = shape.as_shape::<ConvexPolygon<f32>>() {
            self.add_convex(window, object, world, delta, s, color, out)
        } else if let Some(s) = shape.as_shape::<Compound<f32>>() {
            for &(t, ref s) in s.shapes().iter() {
                self.add_shape(window, object, world, delta * t, s.as_ref(), color, out)
            }
        } else if let Some(s) = shape.as_shape::<shape::Polyline<f32>>() {
            self.add_polyline(window, object, world, delta, s, color, out);
        } else {
            println!("Unknown shapes found. It won't be displayed.");
        }
    }

    fn add_plane(
        &mut self,
        window: &mut Window,
        object: ColliderHandle,
        world: &World<f32>,
        shape: &shape::Plane<f32>,
        color: Point3<f32>,
        out: &mut Vec<Node>,
    ) {
        let pos = world.collider(object).unwrap().position();
        let position = Point2::from_coordinates(pos.translation.vector);
        let normal = pos * shape.normal();

        out.push(Node::Plane(Plane::new(
            object, world, &position, &normal, color, window,
        )))
    }

    fn add_polyline(
        &mut self,
        window: &mut Window,
        object: ColliderHandle,
        world: &World<f32>,
        delta: Isometry2<f32>,
        shape: &shape::Polyline<f32>,
        color: Point3<f32>,
        out: &mut Vec<Node>,
    ) {
        let vertices = shape.vertices();

        out.push(Node::Polyline(Polyline::new(
            object,
            world,
            delta,
            vertices.to_vec(),
            color,
            window,
        )))
    }

    fn add_ball(
        &mut self,
        window: &mut Window,
        object: ColliderHandle,
        world: &World<f32>,
        delta: Isometry2<f32>,
        shape: &shape::Ball<f32>,
        color: Point3<f32>,
        out: &mut Vec<Node>,
    ) {
        let margin = world.collider(object).unwrap().data().margin();
        out.push(Node::Ball(Ball::new(
            object,
            world,
            delta,
            shape.radius() + margin,
            color,
            window,
        )))
    }

    fn add_box(
        &mut self,
        window: &mut Window,
        object: ColliderHandle,
        world: &World<f32>,
        delta: Isometry2<f32>,
        shape: &Cuboid<f32>,
        color: Point3<f32>,
        out: &mut Vec<Node>,
    ) {
        let margin = world.collider(object).unwrap().data().margin();
        let rx = shape.half_extents().x + margin;
        let ry = shape.half_extents().y + margin;

        out.push(Node::Box(Box::new(
            object, world, delta, rx, ry, color, window,
        )))
    }

    fn add_convex(
        &mut self,
        window: &mut Window,
        object: ColliderHandle,
        world: &World<f32>,
        delta: Isometry2<f32>,
        shape: &ConvexPolygon<f32>,
        color: Point3<f32>,
        out: &mut Vec<Node>,
    ) {
        let points = shape.points();

        out.push(Node::Convex(Convex::new(
            object,
            world,
            delta,
            points.to_vec(),
            color,
            window,
        )))
    }

    pub fn draw(&mut self, world: &World<f32>, window: &mut Window) {
        for (_, ns) in self.b2sn.iter_mut() {
            for n in ns.iter_mut() {
                n.update(world)
            }
        }

        for (_, ns) in self.b2sn.iter_mut() {
            for n in ns.iter_mut() {
                n.draw(window)
            }
        }
    }

    // pub fn draw_positions(&mut self, window: &mut Window, rbs: &RigidBodies<f32>) {
    //     for (_, ns) in self.b2sn.iter_mut() {
    //         for n in ns.iter_mut() {
    //             let object = n.object();
    //             let rb = rbs.get(object).expect("Rigid body not found.");

    //             // if let WorldObjectBorrowed::RigidBody(rb) = object {
    //                 let t      = rb.position();
    //                 let center = rb.center_of_mass();

    //                 let rotmat = t.rotation.to_rotation_matrix().unwrap();
    //                 let x = rotmat.column(0) * 0.25f32;
    //                 let y = rotmat.column(1) * 0.25f32;
    //                 let z = rotmat.column(2) * 0.25f32;

    //                 window.draw_line(center, &(*center + x), &Point3::new(1.0, 0.0, 0.0));
    //                 window.draw_line(center, &(*center + y), &Point3::new(0.0, 1.0, 0.0));
    //                 window.draw_line(center, &(*center + z), &Point3::new(0.0, 0.0, 1.0));
    //             // }
    //         }
    //     }
    // }

    pub fn camera_mut<'a>(&'a mut self) -> &'a mut PlanarCamera {
        &mut self.camera as &'a mut PlanarCamera
    }

    pub fn look_at(&mut self, at: Point2<f32>, zoom: f32) {
        self.camera.look_at(at, zoom);
    }

    pub fn body_nodes(&self, world: &World<f32>, handle: BodyHandle) -> Option<&Vec<Node>> {
        self.b2sn.get(&Self::body_key(world, handle))
    }

    pub fn body_nodes_mut(
        &mut self,
        world: &World<f32>,
        handle: BodyHandle,
    ) -> Option<&mut Vec<Node>> {
        self.b2sn.get_mut(&Self::body_key(world, handle))
    }
}

impl Default for GraphicsManager {
    fn default() -> Self {
        Self::new()
    }
}
