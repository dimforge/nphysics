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
use nphysics3d::object::RigidBody;
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
            Node::Plane(ref mut n)             => n.select(),
            Node::Ball(ref mut n)              => n.select(),
            Node::Box(ref mut n)               => n.select(),
            Node::Cylinder(ref mut n)          => n.select(),
            Node::Cone(ref mut n)              => n.select(),
            Node::Mesh(ref mut n)              => n.select(),
            Node::Convex(ref mut n)            => n.select()
        }
    }

    pub fn unselect(&mut self) {
        match *self {
            Node::Plane(ref mut n)             => n.unselect(),
            Node::Ball(ref mut n)              => n.unselect(),
            Node::Box(ref mut n)               => n.unselect(),
            Node::Cylinder(ref mut n)          => n.unselect(),
            Node::Cone(ref mut n)              => n.unselect(),
            Node::Mesh(ref mut n)              => n.unselect(),
            Node::Convex(ref mut n)            => n.unselect()
        }
    }

    pub fn update(&mut self) {
        match *self {
            Node::Plane(ref mut n)             => n.update(),
            Node::Ball(ref mut n)              => n.update(),
            Node::Box(ref mut n)               => n.update(),
            Node::Cylinder(ref mut n)          => n.update(),
            Node::Cone(ref mut n)              => n.update(),
            Node::Mesh(ref mut n)              => n.update(),
            Node::Convex(ref mut n)            => n.update()
        }
    }

    pub fn object(&self) -> &SceneNode {
        match *self {
            Node::Plane(ref n)             => n.object(),
            Node::Ball(ref n)              => n.object(),
            Node::Box(ref n)               => n.object(),
            Node::Cylinder(ref n)          => n.object(),
            Node::Cone(ref n)              => n.object(),
            Node::Mesh(ref n)              => n.object(),
            Node::Convex(ref n)            => n.object()
        }
    }

    pub fn body<'a>(&'a self) -> &'a Rc<RefCell<RigidBody<f32>>> {
        match *self {
            Node::Plane(ref n)             => n.body(),
            Node::Ball(ref n)              => n.body(),
            Node::Box(ref n)               => n.body(),
            Node::Cylinder(ref n)          => n.body(),
            Node::Cone(ref n)              => n.body(),
            Node::Mesh(ref n)              => n.body(),
            Node::Convex(ref n)            => n.body()
        }
    }
}

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
                window.remove(&mut sn.object().clone());
            }
        }

        for aabb in self.aabbs.iter_mut() {
            window.remove(aabb);
        }

        self.rb2sn.clear();
        self.aabbs.clear();
    }

    pub fn remove(&mut self, window: &mut Window, body: &Rc<RefCell<RigidBody<f32>>>) {
        let key = &**body as *const RefCell<RigidBody<f32>> as usize;

        match self.rb2sn.get(&key) {
            Some(sns) => {
                for sn in sns.iter() {
                    window.remove(&mut sn.object().clone());
                }
            },
            None => { }
        }

        self.rb2sn.remove(&key);
    }

    pub fn set_color(&mut self, body: &Rc<RefCell<RigidBody<f32>>>, color: Pnt3<f32>) {
        self.rb2color.insert(&**body as *const RefCell<RigidBody<f32>> as usize, color);
    }

    pub fn add(&mut self, window: &mut Window, body: Rc<RefCell<RigidBody<f32>>>) {
        let color;

        match self.rb2color.get(&(&*body as *const RefCell<RigidBody<f32>> as usize)) {
            Some(c) => color = *c,
            None    => {
                if body.borrow().can_move() {
                    color = self.rand.gen();
                }
                else {
                    color = Pnt3::new(0.5, 0.5, 0.5);
                }
            }
        }

        self.add_with_color(window, body, color)
    }

    pub fn add_with_color(&mut self,
                          window: &mut Window,
                          body:   Rc<RefCell<RigidBody<f32>>>,
                          color:  Pnt3<f32>) {
        let nodes = {
            let rb        = body.borrow();
            let mut nodes = Vec::new();

            self.add_repr(window, body.clone(), na::one(), rb.shape_ref(), color, &mut nodes);

            nodes
        };

        self.rb2sn.insert(&*body as *const RefCell<RigidBody<f32>> as usize, nodes);
    }

    fn add_repr(&mut self,
                window: &mut Window,
                body:   Rc<RefCell<RigidBody<f32>>>,
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
            self.add_plane(window, body, s, color, out)
        }
        else if let Some(s) = repr.downcast_ref::<Bl>() {
            self.add_ball(window, body, delta, s, color, out)
        }
        else if let Some(s) = repr.downcast_ref::<Bo>() {
            self.add_box(window, body, delta, s, color, out)
        }
        else if let Some(s) = repr.downcast_ref::<Cx>() {
            self.add_convex(window, body, delta, s, color, out)
        }
        else if let Some(s) = repr.downcast_ref::<Cy>() {
            self.add_cylinder(window, body, delta, s, color, out)
        }
        else if let Some(s) = repr.downcast_ref::<Co>() {
            self.add_cone(window, body, delta, s, color, out)
        }
        else if let Some(s) = repr.downcast_ref::<Cm>() {
            for &(t, ref s) in s.shapes().iter() {
                self.add_repr(window, body.clone(), delta * t, &***s, color, out)
            }
        }
        else if let Some(s) = repr.downcast_ref::<Tm>() {
            self.add_mesh(window, body, delta, s, color, out);
        }
        else {
            panic!("Not yet implemented.")
        }

    }

    fn add_plane(&mut self,
                 window: &mut Window,
                 body:   Rc<RefCell<RigidBody<f32>>>,
                 shape:   &shape::Plane3<f32>,
                 color:  Pnt3<f32>,
                 out:    &mut Vec<Node>) {
        let position = na::translation(body.borrow().position()).translate(&na::orig());
        let normal   = na::rotate(body.borrow().position(), shape.normal());

        out.push(Node::Plane(Plane::new(body, &position, &normal, color, window)))
    }

    fn add_mesh(&mut self,
                window: &mut Window,
                body:   Rc<RefCell<RigidBody<f32>>>,
                delta:  Iso3<f32>,
                shape:   &shape::TriMesh3<f32>,
                color:  Pnt3<f32>,
                out:    &mut Vec<Node>) {
        let vertices = &**shape.vertices();
        let indices = &**shape.indices();

        let is = indices.iter().map(|p| Pnt3::new(p.x as u32, p.y as u32, p.z as u32)).collect();

        out.push(Node::Mesh(Mesh::new(body, delta, vertices.clone(), is, color, window)))
    }

    fn add_ball(&mut self,
                window: &mut Window,
                body:   Rc<RefCell<RigidBody<f32>>>,
                delta:  Iso3<f32>,
                shape:   &shape::Ball3<f32>,
                color:  Pnt3<f32>,
                out:    &mut Vec<Node>) {
        let margin = body.borrow().margin();
        out.push(Node::Ball(Ball::new(body, delta, shape.radius() + margin, color, window)))
    }

    fn add_box(&mut self,
               window: &mut Window,
               body:   Rc<RefCell<RigidBody<f32>>>,
               delta:  Iso3<f32>,
               shape:   &shape::Cuboid3<f32>,
               color:  Pnt3<f32>,
               out:    &mut Vec<Node>) {
        let rx = shape.half_extents().x + body.borrow().margin();
        let ry = shape.half_extents().y + body.borrow().margin();
        let rz = shape.half_extents().z + body.borrow().margin();

        out.push(Node::Box(Box::new(body, delta, rx, ry, rz, color, window)))
    }

    fn add_convex(&mut self,
                  window: &mut Window,
                  body:   Rc<RefCell<RigidBody<f32>>>,
                  delta:  Iso3<f32>,
                  shape:   &shape::Convex3<f32>,
                  color:  Pnt3<f32>,
                  out:    &mut Vec<Node>) {
        out.push(Node::Convex(Convex::new(body, delta, &transformation::convex_hull3(shape.points()), color, window)))
    }

    fn add_cylinder(&mut self,
                    window: &mut Window,
                    body:   Rc<RefCell<RigidBody<f32>>>,
                    delta:  Iso3<f32>,
                    shape:   &shape::Cylinder3<f32>,
                    color:  Pnt3<f32>,
                    out:    &mut Vec<Node>) {
        let r = shape.radius();
        let h = shape.half_height() * 2.0;

        out.push(Node::Cylinder(Cylinder::new(body, delta, r, h, color, window)))
    }

    fn add_cone(&mut self,
                window: &mut Window,
                body:   Rc<RefCell<RigidBody<f32>>>,
                delta:  Iso3<f32>,
                shape:   &shape::Cone3<f32>,
                color:  Pnt3<f32>,
                out:    &mut Vec<Node>) {
        let r = shape.radius();
        let h = shape.half_height() * 2.0;

        out.push(Node::Cone(Cone::new(body, delta, r, h, color, window)))
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
                let rb = n.body().borrow();

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

    pub fn switch_cameras(&mut self) {
        if self.curr_is_arc_ball {
            self.first_person.look_at(self.arc_ball.eye(), self.arc_ball.at());
        }
        else {
            self.arc_ball.look_at(self.first_person.eye(), self.first_person.at());
        }

        self.curr_is_arc_ball = !self.curr_is_arc_ball;
    }

    pub fn camera<'a>(&'a mut self) -> &'a mut Camera {
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

    pub fn body_to_scene_node(&mut self, rb: &Rc<RefCell<RigidBody<f32>>>) -> Option<&mut Vec<Node>> {
        self.rb2sn.get_mut(&(&**rb as *const RefCell<RigidBody<f32>> as usize))
    }
}
