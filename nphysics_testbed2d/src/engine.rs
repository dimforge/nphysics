use camera::Camera;
use na;
use na::{Isometry2, Point3};
use ncollide2d::shape::{self, Compound, ConvexPolygon, Cuboid, Plane, Polyline, Shape};
use ncollide2d::transformation;
use nphysics2d::object::{Body, BodyHandle, ColliderHandle};
use nphysics2d::world::World;
use objects::{Ball, Box, Lines, SceneNode, Segment};
use rand::{Rng, SeedableRng, XorShiftRng};
use sfml::graphics::RenderWindow;
use std::cell::RefCell;
use std::collections::HashMap;
use std::rc::Rc;

pub type GraphicsManagerHandle = Rc<RefCell<GraphicsManager<'static>>>;

pub struct GraphicsManager<'a> {
    // NOTE: sensors and rigid bodies are not on the same hashmap because we want do draw sensors
    // after all the rigid bodies.
    rand: XorShiftRng,
    rb2sn: HashMap<BodyHandle, Vec<SceneNode<'a>>>,
    obj2color: HashMap<BodyHandle, Point3<u8>>,
}

impl<'a> GraphicsManager<'a> {
    pub fn new() -> GraphicsManager<'a> {
        GraphicsManager {
            rand: SeedableRng::from_seed([0, 1, 2, 3]),
            rb2sn: HashMap::new(),
            obj2color: HashMap::new(),
        }
    }

    pub fn add(&mut self, id: ColliderHandle, world: &World<f32>) {
        let collider = world.collider(id).unwrap();
        let parent = collider.data().body();

        // NOTE: not optimal allocation-wise, but it is not critical here.
        let mut new_nodes = Vec::new();
        self.add_shape(id, world, na::one(), &**collider.shape(), &mut new_nodes);

        let nodes = self.rb2sn.entry(parent).or_insert(Vec::new());
        nodes.append(&mut new_nodes);
    }

    fn add_shape(
        &mut self,
        id: ColliderHandle,
        world: &World<f32>,
        delta: Isometry2<f32>,
        shape: &Shape<f32>,
        out: &mut Vec<SceneNode<'a>>,
    ) {
        if let Some(s) = shape.as_shape::<Plane<f32>>() {
            self.add_plane(id, world, s, out)
        } else if let Some(s) = shape.as_shape::<shape::Ball<f32>>() {
            self.add_ball(id, world, delta, s, out)
        } else if let Some(s) = shape.as_shape::<Cuboid<f32>>() {
            self.add_box(id, world, delta, s, out)
        } else if let Some(s) = shape.as_shape::<ConvexPolygon<f32>>() {
            self.add_convex(id, world, delta, s, out)
        } else if let Some(s) = shape.as_shape::<shape::Segment<f32>>() {
            self.add_segment(id, world, delta, s, out)
        } else if let Some(s) = shape.as_shape::<Compound<f32>>() {
            for &(t, ref s) in s.shapes().iter() {
                self.add_shape(id, world.clone(), delta * t, s.as_ref(), out)
            }
        } else if let Some(s) = shape.as_shape::<Polyline<f32>>() {
            self.add_lines(id, world, delta, s, out)
        } else {
            panic!("Not yet implemented.")
        }
    }

    fn add_plane(
        &mut self,
        _: ColliderHandle,
        _: &World<f32>,
        _: &Plane<f32>,
        _: &mut Vec<SceneNode>,
    ) {
    }

    fn add_ball(
        &mut self,
        id: ColliderHandle,
        world: &World<f32>,
        delta: Isometry2<f32>,
        shape: &shape::Ball<f32>,
        out: &mut Vec<SceneNode>,
    ) {
        let collider = world.collider(id).unwrap();
        let color = self.body_color(world, collider.data().body());
        let margin = collider.data().margin();
        out.push(SceneNode::BallNode(Ball::new(
            id,
            world,
            delta,
            shape.radius() + margin,
            color,
        )))
    }

    fn add_convex(
        &mut self,
        id: ColliderHandle,
        world: &World<f32>,
        delta: Isometry2<f32>,
        shape: &ConvexPolygon<f32>,
        out: &mut Vec<SceneNode>,
    ) {
        let color = self.body_color(world, world.collider(id).unwrap().data().body());
        let mut vertices = transformation::convex_hull(shape.points()).unwrap().0;
        let first_pt = vertices[0];
        vertices.push(first_pt); // so it appears closed.

        out.push(SceneNode::LinesNode(Lines::new(
            id,
            world,
            delta,
            vertices,
            color,
        )))
    }

    fn add_lines(
        &mut self,
        id: ColliderHandle,
        world: &World<f32>,
        delta: Isometry2<f32>,
        shape: &Polyline<f32>,
        out: &mut Vec<SceneNode>,
    ) {
        let color = self.body_color(world, world.collider(id).unwrap().data().body());
        let vertices = shape.vertices().to_vec();

        out.push(SceneNode::LinesNode(Lines::new(
            id,
            world,
            delta,
            vertices,
            color,
        )))
    }

    fn add_box(
        &mut self,
        id: ColliderHandle,
        world: &World<f32>,
        delta: Isometry2<f32>,
        shape: &Cuboid<f32>,
        out: &mut Vec<SceneNode>,
    ) {
        let rx = shape.half_extents().x;
        let ry = shape.half_extents().y;

        let collider = world.collider(id).unwrap();
        let margin = collider.data().margin();
        let color = self.body_color(world, collider.data().body());

        out.push(SceneNode::BoxNode(Box::new(
            id,
            world,
            delta,
            rx + margin,
            ry + margin,
            color,
        )))
    }

    fn add_segment(
        &mut self,
        id: ColliderHandle,
        world: &World<f32>,
        delta: Isometry2<f32>,
        shape: &shape::Segment<f32>,
        out: &mut Vec<SceneNode>,
    ) {
        let a = shape.a();
        let b = shape.b();

        let color = self.body_color(world, world.collider(id).unwrap().data().body());

        out.push(SceneNode::SegmentNode(Segment::new(
            id,
            world,
            delta,
            *a,
            *b,
            color,
        )))
    }

    pub fn clear(&mut self) {
        self.rb2sn.clear();
    }

    pub fn draw(&mut self, rw: &mut RenderWindow, c: &Camera, world: &World<f32>) {
        c.activate_scene(rw);

        for (_, ns) in self.rb2sn.iter_mut() {
            for n in ns.iter_mut() {
                match *n {
                    SceneNode::BoxNode(ref mut n) => n.update(world),
                    SceneNode::BallNode(ref mut n) => n.update(world),
                    SceneNode::LinesNode(ref mut n) => n.update(world),
                    SceneNode::SegmentNode(ref mut n) => n.update(world),
                }
            }
        }

        // Draw non-proximity colliders first.
        for (_, ns) in self.rb2sn.iter_mut() {
            for n in ns.iter_mut() {
                if !world
                    .collider(n.collider())
                    .unwrap()
                    .query_type()
                    .is_proximity_query()
                {
                    match *n {
                        SceneNode::BoxNode(ref mut n) => n.draw(rw, world),
                        SceneNode::BallNode(ref mut n) => n.draw(rw, world),
                        SceneNode::LinesNode(ref mut n) => n.draw(rw, world),
                        SceneNode::SegmentNode(ref mut n) => n.draw(rw, world),
                    }
                }
            }
        }

        // Draw proximity colliders.
        for (_, ns) in self.rb2sn.iter_mut() {
            for n in ns.iter_mut() {
                if world
                    .collider(n.collider())
                    .unwrap()
                    .query_type()
                    .is_proximity_query()
                {
                    match *n {
                        SceneNode::BoxNode(ref mut n) => n.draw(rw, world),
                        SceneNode::BallNode(ref mut n) => n.draw(rw, world),
                        SceneNode::LinesNode(ref mut n) => n.draw(rw, world),
                        SceneNode::SegmentNode(ref mut n) => n.draw(rw, world),
                    }
                }
            }
        }

        c.activate_ui(rw);
    }

    fn color_key(world: &World<f32>, handle: BodyHandle) -> BodyHandle {
        if let Body::Multibody(mb) = world.body(handle) {
            mb.handle()
        } else {
            handle
        }
    }

    fn set_color(&mut self, world: &World<f32>, body: BodyHandle, color: Point3<f32>) {
        let color_key = Self::color_key(world, body);
        let color = Point3::new(
            (color.x * 255.0) as u8,
            (color.y * 255.0) as u8,
            (color.z * 255.0) as u8,
        );

        self.obj2color.insert(color_key, color);

        if let Body::Multibody(mb) = world.body(body) {
            for link in mb.links() {
                if let Some(ns) = self.rb2sn.get_mut(&link.handle()) {
                    for n in ns.iter_mut() {
                        n.set_color(color)
                    }
                }
            }
        } else {
            if let Some(ns) = self.rb2sn.get_mut(&color_key) {
                for n in ns.iter_mut() {
                    n.set_color(color)
                }
            }
        }
    }

    pub fn set_body_color(&mut self, world: &World<f32>, body: BodyHandle, color: Point3<f32>) {
        self.set_color(world, body, color)
    }

    pub fn body_color(&mut self, world: &World<f32>, body: BodyHandle) -> Point3<u8> {
        let color_key = Self::color_key(world, body);
        match self.obj2color.get(&color_key) {
            Some(color) => return *color,
            None => {}
        }

        let color = Point3::new(
            self.rand.gen_range(50usize, 256) as u8,
            self.rand.gen_range(50usize, 256) as u8,
            self.rand.gen_range(50usize, 256) as u8,
        );

        self.obj2color.insert(color_key, color);

        color
    }

    pub fn rigid_body_to_scene_node(
        &mut self,
        body: BodyHandle,
    ) -> Option<&mut Vec<SceneNode<'a>>> {
        self.rb2sn.get_mut(&body)
    }
}
