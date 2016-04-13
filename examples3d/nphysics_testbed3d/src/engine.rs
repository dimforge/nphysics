use std::rc::Rc;
use std::cell::RefCell;
use std::collections::HashMap;
use rand::{SeedableRng, XorShiftRng, Rng};
use na::{Pnt3, Iso3, Col, Translate};
use na;
use kiss3d::window::Window;
use kiss3d::scene::SceneNode;
use kiss3d::camera::{Camera, ArcBall, FirstPerson};
use ncollide::shape;
use ncollide::transformation;
use ncollide::inspection::Repr3;
use nphysics3d::object::{RigidBody, WorldObject, WorldObjectBorrowed, RigidBodyHandle, SensorHandle};
use objects::ball::Ball;
use objects::box_node::Box;
use objects::cylinder::Cylinder;
use objects::cone::Cone;
use objects::mesh::Mesh;
use objects::plane::Plane;
use objects::convex::Convex;
use objects::node::Node;

pub type GraphicsManagerHandle = Rc<RefCell<GraphicsManager>>;

pub struct GraphicsManager {
    rand:             XorShiftRng,
    rb2sn:            HashMap<usize, Vec<Node>>,
    rb2color:         HashMap<usize, Pnt3<f32>>,
    arc_ball:         ArcBall,
    first_person:     FirstPerson,
    curr_is_arc_ball: bool,
    aabbs:            Vec<SceneNode>
}

impl GraphicsManager {
    pub fn new() -> GraphicsManager {
        let arc_ball     = ArcBall::new(Pnt3::new(10.0, 10.0, 10.0), Pnt3::new(0.0, 0.0, 0.0));
        let first_person = FirstPerson::new(Pnt3::new(10.0, 10.0, 10.0), Pnt3::new(0.0, 0.0, 0.0));

        let mut rng: XorShiftRng = SeedableRng::from_seed([0, 2, 4, 8]);

        // the first colors are boring.
        for _ in 0usize .. 100 {
            let _: Pnt3<f32> = rng.gen();
        }

        GraphicsManager {
            arc_ball:         arc_ball,
            first_person:     first_person,
            curr_is_arc_ball: true,
            rand:             rng,
            rb2sn:            HashMap::new(),
            rb2color:         HashMap::new(),
            aabbs:            Vec::new()
        }
    }

    pub fn clear(&mut self, window: &mut Window) {
        for sns in self.rb2sn.values() {
            for sn in sns.iter() {
                window.remove(&mut sn.scene_node().clone());
            }
        }

        for aabb in self.aabbs.iter_mut() {
            window.remove(aabb);
        }

        self.rb2sn.clear();
        self.aabbs.clear();
    }

    pub fn remove(&mut self, window: &mut Window, object: &WorldObject<f32>) {
        let key = object.uid();

        match self.rb2sn.get(&key) {
            Some(sns) => {
                for sn in sns.iter() {
                    window.remove(&mut sn.scene_node().clone());
                }
            },
            None => { }
        }

        self.rb2sn.remove(&key);
    }

    pub fn set_rigid_body_color(&mut self, rb: &RigidBodyHandle<f32>, color: Pnt3<f32>) {
        let key = WorldObject::rigid_body_uid(rb);

        self.rb2color.insert(key, color);

        if let Some(ns) = self.rb2sn.get_mut(&key) {
            for n in ns.iter_mut() {
                n.set_color(color)
            }
        }
    }

    pub fn set_sensor_color(&mut self, sensor: &SensorHandle<f32>, color: Pnt3<f32>) {
        let key = WorldObject::sensor_uid(sensor);

        self.rb2color.insert(key, color);

        if let Some(ns) = self.rb2sn.get_mut(&key) {
            for n in ns.iter_mut() {
                n.set_color(color)
            }
        }
    }

    pub fn add(&mut self, window: &mut Window, object: WorldObject<f32>) {
        let mut color = Pnt3::new(0.5, 0.5, 0.5);

        match self.rb2color.get(&object.uid()) {
            Some(c) => color = *c,
            None    => {
                match object {
                    WorldObject::RigidBody(ref rb) => {
                        if rb.borrow().can_move() {
                            color = self.rand.gen();
                        }
                    }
                    WorldObject::Sensor(ref sensor) => {
                        if let Some(parent) = sensor.borrow().parent() {
                            let uid = &**parent as *const RefCell<RigidBody<f32>> as usize;
                            if let Some(pcolor) = self.rb2color.get(&uid) {
                                color = *pcolor;
                            }
                        }
                    }
                }
            }
        }

        self.add_with_color(window, object, color)
    }

    pub fn add_with_color(&mut self, window: &mut Window, object: WorldObject<f32>, color: Pnt3<f32>) {
        let nodes = {
            let mut nodes = Vec::new();

            self.add_repr(window, object.clone(), na::one(), object.borrow().shape().as_ref(), color, &mut nodes);

            nodes
        };

        self.rb2sn.insert(object.uid(), nodes);

        match object {
            WorldObject::RigidBody(ref rb) => {
                self.set_rigid_body_color(&rb, color);
            },
            WorldObject::Sensor(ref sensor) => {
                self.set_sensor_color(&sensor, color);
            }
        }
    }

    fn add_repr(&mut self,
                window: &mut Window,
                object: WorldObject<f32>,
                delta:  Iso3<f32>,
                shape:  &Repr3<f32>,
                color:  Pnt3<f32>,
                out:    &mut Vec<Node>) {
        type Pl = shape::Plane3<f32>;
        type Bl = shape::Ball3<f32>;
        type Bo = shape::Cuboid3<f32>;
        type Cy = shape::Cylinder3<f32>;
        type Co = shape::Cone3<f32>;
        type Cm = shape::Compound3<f32>;
        type Tm = shape::TriMesh3<f32>;
        type Cx = shape::Convex3<f32>;

        let repr = shape.repr();

        if let Some(s) = repr.downcast_ref::<Pl>() {
            self.add_plane(window, object, s, color, out)
        }
        else if let Some(s) = repr.downcast_ref::<Bl>() {
            self.add_ball(window, object, delta, s, color, out)
        }
        else if let Some(s) = repr.downcast_ref::<Bo>() {
            self.add_box(window, object, delta, s, color, out)
        }
        else if let Some(s) = repr.downcast_ref::<Cx>() {
            self.add_convex(window, object, delta, s, color, out)
        }
        else if let Some(s) = repr.downcast_ref::<Cy>() {
            self.add_cylinder(window, object, delta, s, color, out)
        }
        else if let Some(s) = repr.downcast_ref::<Co>() {
            self.add_cone(window, object, delta, s, color, out)
        }
        else if let Some(s) = repr.downcast_ref::<Cm>() {
            for &(t, ref s) in s.shapes().iter() {
                self.add_repr(window, object.clone(), delta * t, s.as_ref(), color, out)
            }
        }
        else if let Some(s) = repr.downcast_ref::<Tm>() {
            self.add_mesh(window, object, delta, s, color, out);
        }
        else {
            panic!("Not yet implemented.")
        }

    }

    fn add_plane(&mut self,
                 window: &mut Window,
                 object: WorldObject<f32>,
                 shape:  &shape::Plane3<f32>,
                 color:  Pnt3<f32>,
                 out:    &mut Vec<Node>) {
        let position = na::translation(&object.borrow().position()).translate(&na::orig());
        let normal   = na::rotate(&object.borrow().position(), shape.normal());

        out.push(Node::Plane(Plane::new(object, &position, &normal, color, window)))
    }

    fn add_mesh(&mut self,
                window: &mut Window,
                object: WorldObject<f32>,
                delta:  Iso3<f32>,
                shape:   &shape::TriMesh3<f32>,
                color:  Pnt3<f32>,
                out:    &mut Vec<Node>) {
        let vertices = &**shape.vertices();
        let indices = &**shape.indices();

        let is = indices.iter().map(|p| Pnt3::new(p.x as u32, p.y as u32, p.z as u32)).collect();

        out.push(Node::Mesh(Mesh::new(object, delta, vertices.clone(), is, color, window)))
    }

    fn add_ball(&mut self,
                window: &mut Window,
                object: WorldObject<f32>,
                delta:  Iso3<f32>,
                shape:   &shape::Ball3<f32>,
                color:  Pnt3<f32>,
                out:    &mut Vec<Node>) {
        let margin = object.borrow().margin();
        out.push(Node::Ball(Ball::new(object, delta, shape.radius() + margin, color, window)))
    }

    fn add_box(&mut self,
               window: &mut Window,
               object: WorldObject<f32>,
               delta:  Iso3<f32>,
               shape:  &shape::Cuboid3<f32>,
               color:  Pnt3<f32>,
               out:    &mut Vec<Node>) {
        let rx = shape.half_extents().x + object.borrow().margin();
        let ry = shape.half_extents().y + object.borrow().margin();
        let rz = shape.half_extents().z + object.borrow().margin();

        out.push(Node::Box(Box::new(object, delta, rx, ry, rz, color, window)))
    }

    fn add_convex(&mut self,
                  window: &mut Window,
                  object: WorldObject<f32>,
                  delta:  Iso3<f32>,
                  shape:   &shape::Convex3<f32>,
                  color:  Pnt3<f32>,
                  out:    &mut Vec<Node>) {
        out.push(Node::Convex(Convex::new(object, delta, &transformation::convex_hull3(shape.points()), color, window)))
    }

    fn add_cylinder(&mut self,
                    window: &mut Window,
                    object: WorldObject<f32>,
                    delta:  Iso3<f32>,
                    shape:   &shape::Cylinder3<f32>,
                    color:  Pnt3<f32>,
                    out:    &mut Vec<Node>) {
        let r = shape.radius();
        let h = shape.half_height() * 2.0;

        out.push(Node::Cylinder(Cylinder::new(object, delta, r, h, color, window)))
    }

    fn add_cone(&mut self,
                window: &mut Window,
                object: WorldObject<f32>,
                delta:  Iso3<f32>,
                shape:   &shape::Cone3<f32>,
                color:  Pnt3<f32>,
                out:    &mut Vec<Node>) {
        let r = shape.radius();
        let h = shape.half_height() * 2.0;

        out.push(Node::Cone(Cone::new(object, delta, r, h, color, window)))
    }

    pub fn draw(&mut self) {
        for (_, ns) in self.rb2sn.iter_mut() {
            for n in ns.iter_mut() {
                n.update()
            }
        }
    }

    pub fn draw_positions(&mut self, window: &mut Window) {
        for (_, ns) in self.rb2sn.iter_mut() {
            for n in ns.iter_mut() {
                let object = n.object().borrow();

                if let WorldObjectBorrowed::RigidBody(rb) = object {
                    let t      = rb.position();
                    let center = rb.center_of_mass();

                    let x = t.rotation.col(0) * 0.25f32;
                    let y = t.rotation.col(1) * 0.25f32;
                    let z = t.rotation.col(2) * 0.25f32;

                    window.draw_line(center, &(*center + x), &Pnt3::new(1.0, 0.0, 0.0));
                    window.draw_line(center, &(*center + y), &Pnt3::new(0.0, 1.0, 0.0));
                    window.draw_line(center, &(*center + z), &Pnt3::new(0.0, 0.0, 1.0));
                }
            }
        }
    }

    pub fn switch_cameras(&mut self) {
        if self.curr_is_arc_ball {
            self.first_person.look_at(self.arc_ball.eye(), self.arc_ball.at());
        }
        else {
            self.arc_ball.look_at(self.first_person.eye(), self.first_person.at());
        }

        self.curr_is_arc_ball = !self.curr_is_arc_ball;
    }

    pub fn camera<'a>(&'a self) -> &'a Camera {
        if self.curr_is_arc_ball {
            &self.arc_ball as &'a Camera
        }
        else {
            &self.first_person as &'a Camera
        }
    }

    pub fn camera_mut<'a>(&'a mut self) -> &'a mut Camera {
        if self.curr_is_arc_ball {
            &mut self.arc_ball as &'a mut Camera
        }
        else {
            &mut self.first_person as &'a mut Camera
        }
    }

    pub fn look_at(&mut self, eye: Pnt3<f32>, at: Pnt3<f32>) {
        self.arc_ball.look_at(eye, at);
        self.first_person.look_at(eye, at);
    }

    pub fn rigid_body_nodes(&self, rb: &RigidBodyHandle<f32>) -> Option<&Vec<Node>> {
        self.rb2sn.get(&WorldObject::rigid_body_uid(rb))
    }

    pub fn sensor_nodes(&self, sensor: &SensorHandle<f32>) -> Option<&Vec<Node>> {
        self.rb2sn.get(&WorldObject::sensor_uid(sensor))
    }

    pub fn rigid_body_nodes_mut(&mut self, rb: &RigidBodyHandle<f32>) -> Option<&mut Vec<Node>> {
        self.rb2sn.get_mut(&WorldObject::rigid_body_uid(rb))
    }

    pub fn sensor_nodes_mut(&mut self, sensor: &SensorHandle<f32>) -> Option<&mut Vec<Node>> {
        self.rb2sn.get_mut(&WorldObject::sensor_uid(sensor))
    }
}
