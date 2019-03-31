use kiss3d::planar_camera::{PlanarCamera, Sidescroll};
use kiss3d::scene::PlanarSceneNode;
use kiss3d::window::Window;
use na;
use na::{Isometry2, Point2, Point3};
use ncollide2d::shape::{self, Compound, ConvexPolygon, Cuboid, Shape};
use ncollide2d::query::Ray;
use ncollide2d::world::CollisionGroups;
use nphysics2d::object::{BodyHandle, BodyPartHandle, ColliderHandle, ColliderAnchor};
use nphysics2d::world::World;
use crate::objects::ball::Ball;
use crate::objects::box_node::Box;
use crate::objects::convex::Convex;
use crate::objects::polyline::Polyline;
use crate::objects::capsule::Capsule;
use crate::objects::heightfield::HeightField;
// use crate::objects::mesh::Mesh;
use crate::objects::node::Node;
use crate::objects::plane::Plane;
use rand::{Rng, SeedableRng, XorShiftRng};
use std::collections::HashMap;

pub struct GraphicsManager {
    rand: XorShiftRng,
    b2sn: HashMap<BodyHandle, Vec<Node>>,
    b2color: HashMap<BodyHandle, Point3<f32>>,
    c2color: HashMap<ColliderHandle, Point3<f32>>,
    rays: Vec<Ray<f32>>,
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
            rays: Vec::new(),
            aabbs: Vec::new(),
        }
    }

    pub fn camera(&self) -> &Sidescroll {
        &self.camera
    }

    pub fn clear(&mut self, window: &mut Window) {
        for sns in self.b2sn.values() {
            for sn in sns.iter() {
                if let Some(n) = sn.scene_node() {
                    window.remove_planar_node(&mut n.clone());
                }
            }
        }

        for aabb in self.aabbs.iter_mut() {
            window.remove_planar_node(aabb);
        }

        self.b2sn.clear();
        self.aabbs.clear();
        self.rays.clear();
    }

    pub fn remove_body_nodes(&mut self, window: &mut Window, body: BodyHandle) {
        if let Some(sns) = self.b2sn.get(&body) {
            for sn in sns.iter() {
                if let Some(n) = sn.scene_node() {
                    window.remove_planar_node(&mut n.clone());
                }
            }
        }

        self.b2sn.remove(&body);
    }

    pub fn remove_body_part_nodes(
        &mut self,
        world: &World<f32>,
        window: &mut Window,
        part: BodyPartHandle,
    ) -> BodyPartHandle {
        let mut delete_array = true;

        if let Some(sns) = self.b2sn.get_mut(&part.0) {
            sns.retain(|sn| {
                if let ColliderAnchor::OnBodyPart {
                    body_part, ..
                } = world.collider(sn.collider()).unwrap().anchor()
                    {
                        if *body_part == part {
                            if let Some(n) = sn.scene_node() {
                                window.remove_planar_node(&mut n.clone());
                            }
                            false
                        } else {
                            delete_array = false;
                            true
                        }
                    } else {
                    delete_array = false;
                    true
                }
            });
        }

        if delete_array {
            self.b2sn.remove(&part.0);
        }

        part
    }

    pub fn update_after_body_key_change(&mut self, world: &World<f32>, body_key: BodyHandle) {
        if let Some(color) = self.b2color.remove(&body_key) {
            if let Some(sns) = self.b2sn.remove(&body_key) {
                for sn in sns {
                    let sn_key = world.collider(sn.collider()).unwrap().body();

                    let _ = self.b2color.entry(sn_key).or_insert(color);
                    let new_sns = self.b2sn.entry(sn_key).or_insert_with(Vec::new);
                    new_sns.push(sn);
                }
            }
        }
    }

    pub fn set_body_color(&mut self, b: BodyHandle, color: Point3<f32>) {
        self.b2color.insert(b, color);

        if let Some(ns) = self.b2sn.get_mut(&b) {
            for n in ns.iter_mut() {
                n.set_color(color)
            }
        }
    }

    pub fn set_collider_color(&mut self, handle: ColliderHandle, color: Point3<f32>) {
        self.c2color.insert(handle, color);
    }

    fn alloc_color(&mut self, handle: BodyHandle) -> Point3<f32> {
        let mut color = Point3::new(0.5, 0.5, 0.5);

        match self.b2color.get(&handle) {
            Some(c) => color = *c,
            None => {
                if !handle.is_ground() {
                    color = self.rand.gen();
                    color *= 1.5;
                    color.x = color.x.min(1.0);
                    color.y = color.y.min(1.0);
                    color.z = color.z.min(1.0);
                }
            }
        }

        self.set_body_color(handle, color);

        color
    }

    pub fn add_ray(&mut self, ray: Ray<f32>) {
        self.rays.push(ray)
    }

    pub fn add(&mut self, window: &mut Window, id: ColliderHandle, world: &World<f32>) {
        let collider = world.collider(id).unwrap();

        let color = if let Some(c) = self.c2color.get(&id).cloned() {
            c
        } else if let Some(c) = self.b2color.get(&collider.body()).cloned() {
            c
        } else {
            self.alloc_color(collider.body())
        };

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
        let key = collider.body();
        let shape = collider.shape().as_ref();

        // NOTE: not optimal allocation-wise, but it is not critical here.
        let mut new_nodes = Vec::new();
        self.add_shape(window, id, world, na::one(), shape, color, &mut new_nodes);

        {
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
        } else if let Some(s) = shape.as_shape::<shape::Capsule<f32>>() {
            self.add_capsule(window, object, world, delta, s, color, out)
        } else if let Some(s) = shape.as_shape::<ConvexPolygon<f32>>() {
            self.add_convex(window, object, world, delta, s, color, out)
        } else if let Some(s) = shape.as_shape::<shape::HeightField<f32>>() {
            self.add_heightfield(window, object, world, delta, s, color, out)
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
        let position = Point2::from(pos.translation.vector);
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
        let vertices = shape.points().to_vec();
        let indices = shape.edges().iter().map(|e| e.indices).collect();

        out.push(Node::Polyline(Polyline::new(
            object,
            world,
            delta,
            vertices,
            indices,
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
        let margin = world.collider(object).unwrap().margin();
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
        let margin = world.collider(object).unwrap().margin();
        let rx = shape.half_extents().x + margin;
        let ry = shape.half_extents().y + margin;

        out.push(Node::Box(Box::new(
            object, world, delta, rx, ry, color, window,
        )))
    }

    fn add_capsule(
        &mut self,
        window: &mut Window,
        object: ColliderHandle,
        world: &World<f32>,
        delta: Isometry2<f32>,
        shape: &shape::Capsule<f32>,
        color: Point3<f32>,
        out: &mut Vec<Node>,
    ) {
        let margin = world.collider(object).unwrap().margin();
        let r = shape.radius() + margin;
        let hh = shape.half_height();

        out.push(Node::Capsule(Capsule::new(
            object, world, delta, r, hh, color, window,
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

    fn add_heightfield(
        &mut self,
        window: &mut Window,
        object: ColliderHandle,
        world: &World<f32>,
        delta: Isometry2<f32>,
        heightfield: &shape::HeightField<f32>,
        color: Point3<f32>,
        out: &mut Vec<Node>,
    ) {
        out.push(Node::HeightField(HeightField::new(
            object,
            world,
            delta,
            heightfield,
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

        for ray in &self.rays {
            let groups = CollisionGroups::new();
            let inter = world.collider_world().interferences_with_ray(ray, &groups);
            let hit = inter.fold(1000.0, |t, hit| hit.1.toi.min(t));
            let p1 = ray.origin;
            let p2 = ray.origin + ray.dir * hit;
            window.draw_planar_line(&p1, &p2, &Point3::new(1.0, 0.0, 0.0));
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

    pub fn body_nodes(&self, handle: BodyHandle) -> Option<&Vec<Node>> {
        self.b2sn.get(&handle)
    }

    pub fn body_nodes_mut(
        &mut self,
        handle: BodyHandle,
    ) -> Option<&mut Vec<Node>> {
        self.b2sn.get_mut(&handle)
    }
}

impl Default for GraphicsManager {
    fn default() -> Self {
        Self::new()
    }
}
