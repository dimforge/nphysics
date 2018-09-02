use kiss3d::camera::{ArcBall, Camera, FirstPerson};
use kiss3d::scene::SceneNode;
use kiss3d::window::Window;
use na;
use na::{Isometry3, Point3};
use ncollide3d::shape::{self, Compound, ConvexHull, Cuboid, Shape, TriMesh};
use ncollide3d::transformation;
use nphysics3d::object::{BodyHandle, BodyPartHandle, ColliderHandle, DeformableVolume};
use nphysics3d::world::World;
use objects::ball::Ball;
use objects::box_node::Box;
use objects::convex::Convex;
use objects::mesh::Mesh;
use objects::node::Node;
use objects::deformable_mesh::DeformableMesh;
use objects::plane::Plane;
use rand::{Rng, SeedableRng, XorShiftRng};
use std::collections::HashMap;

pub struct GraphicsManager {
    rand: XorShiftRng,
    b2sn: HashMap<BodyHandle, Vec<Node>>,
    b2color: HashMap<BodyHandle, Point3<f32>>,
    c2color: HashMap<ColliderHandle, Point3<f32>>,
    arc_ball: ArcBall,
    first_person: FirstPerson,
    curr_is_arc_ball: bool,
    aabbs: Vec<SceneNode>,
}

impl GraphicsManager {
    pub fn new() -> GraphicsManager {
        let arc_ball = ArcBall::new(Point3::new(10.0, 10.0, 10.0), Point3::new(0.0, 0.0, 0.0));
        let first_person =
            FirstPerson::new(Point3::new(10.0, 10.0, 10.0), Point3::new(0.0, 0.0, 0.0));

        let mut rng: XorShiftRng = SeedableRng::from_seed([0; 16]);

        // the first colors are boring.
        for _ in 0usize..100 {
            let _: Point3<f32> = rng.gen();
        }

        GraphicsManager {
            arc_ball: arc_ball,
            first_person: first_person,
            curr_is_arc_ball: true,
            rand: rng,
            b2sn: HashMap::new(),
            b2color: HashMap::new(),
            c2color: HashMap::new(),
            aabbs: Vec::new(),
        }
    }

    pub fn clear(&mut self, window: &mut Window) {
        for sns in self.b2sn.values() {
            for sn in sns.iter() {
                window.remove_node(&mut sn.scene_node().clone());
            }
        }

        for aabb in self.aabbs.iter_mut() {
            window.remove_node(aabb);
        }

        self.b2sn.clear();
        self.aabbs.clear();
    }

    pub fn remove_body_nodes(&mut self, window: &mut Window, body: BodyHandle) {
        if let Some(sns) = self.b2sn.get(&body) {
            for sn in sns.iter() {
                window.remove_node(&mut sn.scene_node().clone());
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

        if let Some(sns) = self.b2sn.get_mut(&part.body_handle) {
            sns.retain(|sn| {
                if world.collider(sn.collider()).unwrap().data().body_part() == part {
                    window.remove_node(&mut sn.scene_node().clone());
                    false
                } else {
                    delete_array = false;
                    true
                }
            });
        }

        if delete_array {
            self.b2sn.remove(&part.body_handle);
        }

        part
    }

    pub fn update_after_body_key_change(&mut self, world: &World<f32>, body_key: BodyHandle) {
        if let Some(color) = self.b2color.remove(&body_key) {
            if let Some(sns) = self.b2sn.remove(&body_key) {
                for sn in sns {
                    let sn_body = world.collider(sn.collider()).unwrap().data().body_part();
                    let sn_key = sn_body.body_handle;

                    let _ = self.b2color.entry(sn_key).or_insert(color);
                    let new_sns = self.b2sn.entry(sn_key).or_insert(Vec::new());
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

    pub fn add_deformable_volume(&mut self, window: &mut Window, handle: BodyHandle, world: &World<f32>) {
        if let Some(body) = world.body(handle).downcast_ref::<DeformableVolume<f32>>().ok() {
            let color = self.alloc_color(handle);
            let boundary = body.boundary();
            let vertices = body.positions().as_slice().chunks(3).map(|p| Point3::new(p[0], p[1], p[2])).collect();
            let indices = body.boundary().into_iter().map(|p| Point3::new((p.x / 3) as u16, (p.y / 3) as u16, (p.z / 3) as u16)).collect();

            let node = Node::DeformableMesh(DeformableMesh::new(
                handle,
                world,
                vertices,
                indices,
                color,
                window,
            ));

            let nodes = self.b2sn.entry(handle).or_insert(Vec::new());
            nodes.push(node);
        }
    }

    pub fn add(&mut self, window: &mut Window, id: ColliderHandle, world: &World<f32>) {
        let collider = world.collider(id).unwrap();

        let color;
        if let Some(c) = self.c2color.get(&id).cloned() {
            color = c;
            self.set_body_color(collider.data().body_part().body_handle, color);
        } else {
            color = self.alloc_color(collider.data().body_part().body_handle)
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
        let parent = collider.data().body_part();
        let shape = collider.shape().as_ref();

        // NOTE: not optimal allocation-wise, but it is not critical here.
        let mut new_nodes = Vec::new();
        self.add_shape(window, id, world, na::one(), shape, color, &mut new_nodes);

        {
            let key = parent.body_handle;
            let nodes = self.b2sn.entry(key).or_insert(Vec::new());
            nodes.append(&mut new_nodes);
        }
    }

    fn add_shape(
        &mut self,
        window: &mut Window,
        object: ColliderHandle,
        world: &World<f32>,
        delta: Isometry3<f32>,
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
        } else if let Some(s) = shape.as_shape::<ConvexHull<f32>>() {
            self.add_convex(window, object, world, delta, s, color, out) /*
        } else if let Some(s) = shape.as_shape::<shape::Cylinder<f32>>() {
            self.add_cylinder(window, object, world, delta, s, color, out)
        } else if let Some(s) = shape.as_shape::<shape::Cone<f32>>() {
            self.add_cone(window, object, world, delta, s, color, out)*/
        } else if let Some(s) = shape.as_shape::<Compound<f32>>() {
            for &(t, ref s) in s.shapes().iter() {
                self.add_shape(window, object, world, delta * t, s.as_ref(), color, out)
            }
        } else if let Some(s) = shape.as_shape::<TriMesh<f32>>() {
            self.add_mesh(window, object, world, delta, s, color, out);
        } else {
            panic!("Not yet implemented.")
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
        let position = Point3::from_coordinates(pos.translation.vector);
        let normal = pos * shape.normal();

        out.push(Node::Plane(Plane::new(
            object, world, &position, &normal, color, window,
        )))
    }

    fn add_mesh(
        &mut self,
        window: &mut Window,
        object: ColliderHandle,
        world: &World<f32>,
        delta: Isometry3<f32>,
        shape: &TriMesh<f32>,
        color: Point3<f32>,
        out: &mut Vec<Node>,
    ) {
        let vertices = shape.vertices();
        let indices = shape.indices();

        let is = indices
            .iter()
            .map(|p| Point3::new(p.x as u32, p.y as u32, p.z as u32))
            .collect();

        out.push(Node::Mesh(Mesh::new(
            object,
            world,
            delta,
            vertices.clone(),
            is,
            color,
            window,
        )))
    }

    fn add_ball(
        &mut self,
        window: &mut Window,
        object: ColliderHandle,
        world: &World<f32>,
        delta: Isometry3<f32>,
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
        delta: Isometry3<f32>,
        shape: &Cuboid<f32>,
        color: Point3<f32>,
        out: &mut Vec<Node>,
    ) {
        let margin = world.collider(object).unwrap().data().margin();
        let rx = shape.half_extents().x + margin;
        let ry = shape.half_extents().y + margin;
        let rz = shape.half_extents().z + margin;

        out.push(Node::Box(Box::new(
            object, world, delta, rx, ry, rz, color, window,
        )))
    }

    fn add_convex(
        &mut self,
        window: &mut Window,
        object: ColliderHandle,
        world: &World<f32>,
        delta: Isometry3<f32>,
        shape: &ConvexHull<f32>,
        color: Point3<f32>,
        out: &mut Vec<Node>,
    ) {
        let mut chull = transformation::convex_hull(shape.points());
        chull.replicate_vertices();
        chull.recompute_normals();

        out.push(Node::Convex(Convex::new(
            object, world, delta, &chull, color, window,
        )))
    }

    pub fn draw(&mut self, world: &World<f32>) {
        for (_, ns) in self.b2sn.iter_mut() {
            for n in ns.iter_mut() {
                n.update(world)
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

    pub fn switch_cameras(&mut self) {
        if self.curr_is_arc_ball {
            self.first_person
                .look_at(self.arc_ball.eye(), self.arc_ball.at());
        } else {
            self.arc_ball
                .look_at(self.first_person.eye(), self.first_person.at());
        }

        self.curr_is_arc_ball = !self.curr_is_arc_ball;
    }

    pub fn camera<'a>(&'a self) -> &'a Camera {
        if self.curr_is_arc_ball {
            &self.arc_ball as &'a Camera
        } else {
            &self.first_person as &'a Camera
        }
    }

    pub fn camera_mut<'a>(&'a mut self) -> &'a mut Camera {
        if self.curr_is_arc_ball {
            &mut self.arc_ball as &'a mut Camera
        } else {
            &mut self.first_person as &'a mut Camera
        }
    }

    pub fn look_at(&mut self, eye: Point3<f32>, at: Point3<f32>) {
        self.arc_ball.look_at(eye, at);
        self.first_person.look_at(eye, at);
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
