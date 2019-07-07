#[cfg(feature = "dim2")]
use kiss3d::planar_camera::Sidescroll as Camera;
#[cfg(feature = "dim3")]
use kiss3d::camera::ArcBall as Camera;
use kiss3d::window::Window;
use na;
use na::Point3;
use ncollide::shape::{self, Compound, Cuboid, Shape};
#[cfg(feature = "dim2")]
use na::Translation2 as Translation;
#[cfg(feature = "dim3")]
use na::Translation3 as Translation;

#[cfg(feature = "dim2")]
use ncollide::shape::ConvexPolygon;
#[cfg(feature = "dim3")]
use ncollide::shape::{ConvexHull, TriMesh};
#[cfg(feature = "dim3")]
use ncollide::transformation;
use ncollide::query::Ray;
use ncollide::pipeline::object::CollisionGroups;
use nphysics::object::{
    DefaultBodyHandle, DefaultBodyPartHandle, DefaultColliderHandle,
    ColliderAnchor, DefaultColliderSet
};
use nphysics::world::DefaultColliderWorld;
use nphysics::math::{Isometry, Vector, Point};
use crate::objects::ball::Ball;
use crate::objects::box_node::Box;
use crate::objects::convex::Convex;
#[cfg(feature = "dim3")]
use crate::objects::mesh::Mesh;
#[cfg(feature = "dim2")]
use crate::objects::polyline::Polyline;
use crate::objects::node::{GraphicsNode, Node};
use crate::objects::heightfield::HeightField;
use crate::objects::plane::Plane;
use crate::objects::capsule::Capsule;
use rand::{Rng, SeedableRng, rngs::StdRng};
use std::collections::HashMap;

pub trait GraphicsWindow {
    fn remove_graphics_node(&mut self, node: &mut GraphicsNode);
    fn draw_graphics_line(&mut self, p1: &Point<f32>, p2: &Point<f32>, color: &Point3<f32>);
}

impl GraphicsWindow for Window {
    fn remove_graphics_node(&mut self, node: &mut GraphicsNode) {
        #[cfg(feature = "dim2")]
            self.remove_planar_node(node);
        #[cfg(feature = "dim3")]
            self.remove_node(node);
    }

    fn draw_graphics_line(&mut self, p1: &Point<f32>, p2: &Point<f32>, color: &Point3<f32>) {
        #[cfg(feature = "dim2")]
            self.draw_planar_line(p1, p2, color);
        #[cfg(feature = "dim3")]
            self.draw_line(p1, p2, color);
    }

}

pub struct GraphicsManager {
    rand: StdRng,
    b2sn: HashMap<DefaultBodyHandle, Vec<Node>>,
    b2color: HashMap<DefaultBodyHandle, Point3<f32>>,
    c2color: HashMap<DefaultColliderHandle, Point3<f32>>,
    rays: Vec<Ray<f32>>,
    camera: Camera,
    aabbs: Vec<(DefaultColliderHandle, GraphicsNode)>,
}

impl GraphicsManager {
    pub fn new() -> GraphicsManager {
        let mut camera;

        #[cfg(feature = "dim3")]
            {
                camera = Camera::new(Point3::new(10.0, 10.0, 10.0), Point3::new(0.0, 0.0, 0.0));
                camera.set_rotate_modifiers(Some(kiss3d::event::Modifiers::Control));
            }

        #[cfg(feature = "dim2")]
            {
                camera = Camera::new();
                camera.set_zoom(50.0);
            }

        GraphicsManager {
            camera,
            rand: StdRng::seed_from_u64(0),
            b2sn: HashMap::new(),
            b2color: HashMap::new(),
            c2color: HashMap::new(),
            rays: Vec::new(),
            aabbs: Vec::new(),
        }
    }

    pub fn clear(&mut self, window: &mut Window) {
        for sns in self.b2sn.values_mut() {
            for sn in sns.iter_mut() {
                if let Some(node) = sn.scene_node_mut() {
                    window.remove_graphics_node(node);
                }
            }
        }

        for aabb in self.aabbs.iter_mut() {
            window.remove_graphics_node(&mut aabb.1);
        }

        self.b2sn.clear();
        self.aabbs.clear();
        self.rays.clear();
        self.b2color.clear();
        self.c2color.clear();
        self.rand = StdRng::seed_from_u64(0);
    }

    pub fn remove_body_nodes(&mut self, window: &mut Window, body: DefaultBodyHandle) {
        if let Some(sns) = self.b2sn.get_mut(&body) {
            for sn in sns.iter_mut() {
                if let Some(node) = sn.scene_node_mut() {
                    window.remove_graphics_node(node);
                }
            }
        }

        self.b2sn.remove(&body);
    }

    pub fn remove_body_part_nodes(
        &mut self,
        colliders: &DefaultColliderSet<f32>,
        window: &mut Window,
        part: DefaultBodyPartHandle,
    ) -> DefaultBodyPartHandle {
        let mut delete_array = true;

        if let Some(sns) = self.b2sn.get_mut(&part.0) {
            sns.retain(|sn| {
                if let ColliderAnchor::OnBodyPart {
                    body_part, ..
                } = colliders.get(sn.collider()).unwrap().anchor()
                    {
                        if *body_part == part {
                            if let Some(node) = sn.scene_node() {
                                window.remove_graphics_node(&mut node.clone());
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

    pub fn update_after_body_key_change(&mut self, colliders: &DefaultColliderSet<f32>, body_key: DefaultBodyHandle) {
        if let Some(color) = self.b2color.remove(&body_key) {
            if let Some(sns) = self.b2sn.remove(&body_key) {
                for sn in sns {
                    let sn_key = colliders.get(sn.collider()).unwrap().body();

                    let _ = self.b2color.entry(sn_key).or_insert(color);
                    let new_sns = self.b2sn.entry(sn_key).or_insert_with(Vec::new);
                    new_sns.push(sn);
                }
            }
        }
    }

    pub fn set_body_color(&mut self, b: DefaultBodyHandle, color: Point3<f32>) {
        self.b2color.insert(b, color);

        if let Some(ns) = self.b2sn.get_mut(&b) {
            for n in ns.iter_mut() {
                n.set_color(color)
            }
        }
    }

    pub fn set_collider_color(&mut self, handle: DefaultColliderHandle, color: Point3<f32>) {
        self.c2color.insert(handle, color);
    }

    fn alloc_color(&mut self, handle: DefaultBodyHandle) -> Point3<f32> {
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

    pub fn add(&mut self, window: &mut Window, id: DefaultColliderHandle, colliders: &DefaultColliderSet<f32>) {
        let collider = colliders.get(id).unwrap();

        let color = if let Some(c) = self.c2color.get(&id).cloned() {
            c
        } else if let Some(c) = self.b2color.get(&collider.body()).cloned() {
            c
        } else {
            self.alloc_color(collider.body())
        };

        self.add_with_color(window, id, colliders, color)
    }

    pub fn add_with_color(
        &mut self,
        window: &mut Window,
        id: DefaultColliderHandle,
        colliders: &DefaultColliderSet<f32>,
        color: Point3<f32>,
    ) {
        let collider = colliders.get(id).unwrap();
        let key = collider.body();
        let shape = collider.shape().as_ref();

        // NOTE: not optimal allocation-wise, but it is not critical here.
        let mut new_nodes = Vec::new();
        self.add_shape(window, id, colliders, na::one(), shape, color, &mut new_nodes);

        {
            let nodes = self.b2sn.entry(key).or_insert_with(Vec::new);
            nodes.append(&mut new_nodes);
        }
    }

    fn add_shape(
        &mut self,
        window: &mut Window,
        object: DefaultColliderHandle,
        colliders: &DefaultColliderSet<f32>,
        delta: Isometry<f32>,
        shape: &Shape<f32>,
        color: Point3<f32>,
        out: &mut Vec<Node>,
    ) {
        if let Some(s) = shape.as_shape::<shape::Plane<f32>>() {
            self.add_plane(window, object, colliders, s, color, out)
        } else if let Some(s) = shape.as_shape::<shape::Ball<f32>>() {
            self.add_ball(window, object, colliders, delta, s, color, out)
        } else if let Some(s) = shape.as_shape::<Cuboid<f32>>() {
            self.add_box(window, object, colliders, delta, s, color, out)
        } else if let Some(s) = shape.as_shape::<shape::Capsule<f32>>() {
            self.add_capsule(window, object, colliders, delta, s, color, out)
        } else if let Some(s) = shape.as_shape::<Compound<f32>>() {
            for &(t, ref s) in s.shapes().iter() {
                self.add_shape(window, object, colliders, delta * t, s.as_ref(), color, out)
            }
        }

        #[cfg(feature = "dim2")]
            {
                if let Some(s) = shape.as_shape::<ConvexPolygon<f32>>() {
                    self.add_convex(window, object, colliders, delta, s, color, out)
                } else if let Some(s) = shape.as_shape::<shape::Polyline<f32>>() {
                    self.add_polyline(window, object, colliders, delta, s, color, out);
                } else if let Some(s) = shape.as_shape::<shape::HeightField<f32>>() {
                    self.add_heightfield(window, object, colliders, delta, s, color, out);
                }
            }

        #[cfg(feature = "dim3")]
            {
                if let Some(s) = shape.as_shape::<ConvexHull<f32>>() {
                    self.add_convex(window, object, colliders, delta, s, color, out)
                } else if let Some(s) = shape.as_shape::<TriMesh<f32>>() {
                    self.add_mesh(window, object, colliders, delta, s, color, out);
                } else if let Some(s) = shape.as_shape::<shape::HeightField<f32>>() {
                    self.add_heightfield(window, object, colliders, delta, s, color, out);
                }
            }
    }

    fn add_plane(
        &mut self,
        window: &mut Window,
        object: DefaultColliderHandle,
        colliders: &DefaultColliderSet<f32>,
        shape: &shape::Plane<f32>,
        color: Point3<f32>,
        out: &mut Vec<Node>,
    ) {
        let pos = colliders.get(object).unwrap().position();
        let position = Point::from(pos.translation.vector);
        let normal = pos * shape.normal();

        out.push(Node::Plane(Plane::new(
            object, colliders, &position, &normal, color, window,
        )))
    }

    #[cfg(feature = "dim2")]
    fn add_polyline(
        &mut self,
        window: &mut Window,
        object: DefaultColliderHandle,
        colliders: &DefaultColliderSet<f32>,
        delta: Isometry<f32>,
        shape: &shape::Polyline<f32>,
        color: Point3<f32>,
        out: &mut Vec<Node>,
    ) {
        let vertices = shape.points().to_vec();
        let indices = shape.edges().iter().map(|e| e.indices).collect();

        out.push(Node::Polyline(Polyline::new(
            object,
            colliders,
            delta,
            vertices,
            indices,
            color,
            window,
        )))
    }

    #[cfg(feature = "dim3")]
    fn add_mesh(
        &mut self,
        window: &mut Window,
        object: DefaultColliderHandle,
        colliders: &DefaultColliderSet<f32>,
        delta: Isometry<f32>,
        shape: &TriMesh<f32>,
        color: Point3<f32>,
        out: &mut Vec<Node>,
    ) {
        let points = shape.points();
        let faces = shape.faces();

        let is = faces
            .iter()
            .map(|f| Point3::new(f.indices.x as u32, f.indices.y as u32, f.indices.z as u32))
            .collect();

        out.push(Node::Mesh(Mesh::new(
            object,
            colliders,
            delta,
            points.to_vec(),
            is,
            color,
            window,
        )))
    }

    fn add_heightfield(
        &mut self,
        window: &mut Window,
        object: DefaultColliderHandle,
        colliders: &DefaultColliderSet<f32>,
        delta: Isometry<f32>,
        heightfield: &shape::HeightField<f32>,
        color: Point3<f32>,
        out: &mut Vec<Node>,
    ) {
        out.push(Node::HeightField(HeightField::new(
            object,
            colliders,
            delta,
            heightfield,
            color,
            window,
        )))
    }

    fn add_capsule(
        &mut self,
        window: &mut Window,
        object: DefaultColliderHandle,
        colliders: &DefaultColliderSet<f32>,
        delta: Isometry<f32>,
        shape: &shape::Capsule<f32>,
        color: Point3<f32>,
        out: &mut Vec<Node>,
    ) {
        let margin = colliders.get(object).unwrap().margin();
        out.push(Node::Capsule(Capsule::new(
            object,
            colliders,
            delta,
            shape.radius() + margin,
            shape.height(),
            color,
            window,
        )))
    }

    fn add_ball(
        &mut self,
        window: &mut Window,
        object: DefaultColliderHandle,
        colliders: &DefaultColliderSet<f32>,
        delta: Isometry<f32>,
        shape: &shape::Ball<f32>,
        color: Point3<f32>,
        out: &mut Vec<Node>,
    ) {
        let margin = colliders.get(object).unwrap().margin();
        out.push(Node::Ball(Ball::new(
            object,
            colliders,
            delta,
            shape.radius() + margin,
            color,
            window,
        )))
    }

    fn add_box(
        &mut self,
        window: &mut Window,
        object: DefaultColliderHandle,
        colliders: &DefaultColliderSet<f32>,
        delta: Isometry<f32>,
        shape: &Cuboid<f32>,
        color: Point3<f32>,
        out: &mut Vec<Node>,
    ) {
        let margin = colliders.get(object).unwrap().margin();

        out.push(Node::Box(Box::new(
            object, colliders, delta, shape.half_extents() + Vector::repeat(margin), color, window,
        )))
    }

    #[cfg(feature = "dim2")]
    fn add_convex(
        &mut self,
        window: &mut Window,
        object: DefaultColliderHandle,
        colliders: &DefaultColliderSet<f32>,
        delta: Isometry<f32>,
        shape: &ConvexPolygon<f32>,
        color: Point3<f32>,
        out: &mut Vec<Node>,
    ) {
        let points = shape.points();

        out.push(Node::Convex(Convex::new(
            object,
            colliders,
            delta,
            points.to_vec(),
            color,
            window,
        )))
    }

    #[cfg(feature = "dim3")]
    fn add_convex(
        &mut self,
        window: &mut Window,
        object: DefaultColliderHandle,
        colliders: &DefaultColliderSet<f32>,
        delta: Isometry<f32>,
        shape: &ConvexHull<f32>,
        color: Point3<f32>,
        out: &mut Vec<Node>,
    ) {
        let mut chull = transformation::convex_hull(shape.points());
        chull.replicate_vertices();
        chull.recompute_normals();

        out.push(Node::Convex(Convex::new(
            object, colliders, delta, &chull, color, window,
        )))
    }

    pub fn show_aabbs(&mut self, collider_world: &DefaultColliderWorld<f32>, colliders: &DefaultColliderSet<f32>, window: &mut Window) {
        for (_, ns) in self.b2sn.iter() {
            for n in ns.iter() {
                let handle = n.collider();
                if let Some(collider) = colliders.get(handle) {
                    let color = if let Some(c) = self.c2color.get(&handle).cloned() {
                        c
                    } else {
                        self.b2color[&collider.body()]
                    };

                    #[cfg(feature = "dim2")]
                        let mut cube = window.add_rectangle(1.0, 1.0);
                    #[cfg(feature = "dim3")]
                        let mut cube = window.add_cube(1.0, 1.0, 1.0);
                    cube.set_surface_rendering_activation(false);
                    cube.set_lines_width(5.0);
                    cube.set_color(color.x, color.y, color.z);
                    self.aabbs.push((handle, cube));
                }
            }
        }
    }

    pub fn hide_aabbs(&mut self, window: &mut Window) {
        for mut aabb in self.aabbs.drain(..) {
            window.remove_graphics_node(&mut aabb.1)
        }
    }

    pub fn draw(&mut self, collider_world: &DefaultColliderWorld<f32>, colliders: &DefaultColliderSet<f32>, window: &mut Window) {
        for (_, ns) in self.b2sn.iter_mut() {
            for n in ns.iter_mut() {
                n.update(colliders)
            }
        }

        for (_, ns) in self.b2sn.iter_mut() {
            for n in ns.iter_mut() {
                n.draw(window)
            }
        }

        for (handle, node) in &mut self.aabbs {
            use ncollide::pipeline::object::CollisionObjectRef;

            if let Some(collider) = colliders.get(*handle) {
                let bf = collider_world.broad_phase();
                let aabb = collider
                    .proxy_handle()
                    .and_then(|h| bf.proxy(h))
                    .map(|p| p.0);

                if let Some(aabb) = aabb {
                    let w = aabb.half_extents() * 2.0;

                    node.set_local_translation(Translation::from(aabb.center().coords));

                    #[cfg(feature = "dim2")]
                        node.set_local_scale(w.x, w.y);
                    #[cfg(feature = "dim3")]
                        node.set_local_scale(w.x, w.y, w.z);
                }
            }
        }

        for ray in &self.rays {
            let groups = CollisionGroups::new();
            let inter = collider_world.interferences_with_ray(colliders, ray, &groups);
            let hit = inter.fold(1000.0, |t, hit| hit.1.toi.min(t));
            let p1 = ray.origin;
            let p2 = ray.origin + ray.dir * hit;
            window.draw_graphics_line(&p1, &p2, &Point3::new(1.0, 0.0, 0.0));
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

    pub fn camera(&self) -> &Camera {
        &self.camera
    }

    pub fn camera_mut(&mut self) -> &mut Camera {
        &mut self.camera
    }

    #[cfg(feature = "dim3")]
    pub fn look_at(&mut self, eye: Point<f32>, at: Point<f32>) {
        self.camera.look_at(eye, at);
    }

    #[cfg(feature = "dim2")]
    pub fn look_at(&mut self, at: Point<f32>, zoom: f32) {
        self.camera.look_at(at, zoom);
    }

    pub fn body_nodes(&self, handle: DefaultBodyHandle) -> Option<&Vec<Node>> {
        self.b2sn.get(&handle)
    }

    pub fn body_nodes_mut(
        &mut self,
        handle: DefaultBodyHandle,
    ) -> Option<&mut Vec<Node>> {
        self.b2sn.get_mut(&handle)
    }

    pub fn nodes(&self) -> impl Iterator<Item = &Node> {
        self.b2sn.values().flat_map(|val| val.iter())
    }

    pub fn nodes_mut(&mut self) -> impl Iterator<Item = &mut Node> {
        self.b2sn.values_mut().flat_map(|val| val.iter_mut())
    }
}

impl Default for GraphicsManager {
    fn default() -> Self {
        Self::new()
    }
}
