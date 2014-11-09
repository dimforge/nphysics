use std::intrinsics::TypeId;
use std::any::AnyRefExt;
use std::rc::Rc;
use std::cell::RefCell;
use std::num::One;
use std::collections::HashMap;
use rand::{SeedableRng, XorShiftRng, Rng};
use na::{Pnt3, Vec3, Iso3, Col, Translate};
use na;
use kiss3d::window::Window;
use kiss3d::scene::SceneNode;
use kiss3d::camera::{Camera, ArcBall, FirstPerson};
use ncollide::shape::Shape3;
use ncollide::shape;
use ncollide::procedural;
use nphysics::object::RigidBody;
use objects::bezier_surface::BezierSurface;
use objects::ball::Ball;
use objects::box_node::Box;
use objects::cylinder::Cylinder;
use objects::cone::Cone;
use objects::mesh::Mesh;
use objects::plane::Plane;
use objects::convex::Convex;


pub enum Node {
    BallNode(Ball),
    BoxNode(Box),
    CylinderNode(Cylinder),
    ConeNode(Cone),
    MeshNode(Mesh),
    PlaneNode(Plane),
    BezierSurfaceNode(BezierSurface),
    ConvexNode(Convex)
}

impl Node {
    pub fn select(&mut self) {
        match *self {
            PlaneNode(ref mut n)             => n.select(),
            BallNode(ref mut n)              => n.select(),
            BoxNode(ref mut n)               => n.select(),
            CylinderNode(ref mut n)          => n.select(),
            ConeNode(ref mut n)              => n.select(),
            MeshNode(ref mut n)              => n.select(),
            BezierSurfaceNode(ref mut n)     => n.select(),
            ConvexNode(ref mut n)            => n.select()
        }
    }

    pub fn unselect(&mut self) {
        match *self {
            PlaneNode(ref mut n)             => n.unselect(),
            BallNode(ref mut n)              => n.unselect(),
            BoxNode(ref mut n)               => n.unselect(),
            CylinderNode(ref mut n)          => n.unselect(),
            ConeNode(ref mut n)              => n.unselect(),
            MeshNode(ref mut n)              => n.unselect(),
            BezierSurfaceNode(ref mut n)     => n.unselect(),
            ConvexNode(ref mut n)            => n.unselect()
        }
    }

    pub fn update(&mut self) {
        match *self {
            PlaneNode(ref mut n)             => n.update(),
            BallNode(ref mut n)              => n.update(),
            BoxNode(ref mut n)               => n.update(),
            CylinderNode(ref mut n)          => n.update(),
            ConeNode(ref mut n)              => n.update(),
            MeshNode(ref mut n)              => n.update(),
            BezierSurfaceNode(ref mut n)     => n.update(),
            ConvexNode(ref mut n)            => n.update()
        }
    }

    pub fn object(&self) -> &SceneNode {
        match *self {
            PlaneNode(ref n)             => n.object(),
            BallNode(ref n)              => n.object(),
            BoxNode(ref n)               => n.object(),
            CylinderNode(ref n)          => n.object(),
            ConeNode(ref n)              => n.object(),
            MeshNode(ref n)              => n.object(),
            BezierSurfaceNode(ref n)     => n.object(),
            ConvexNode(ref n)            => n.object()
        }
    }

    pub fn body<'a>(&'a self) -> &'a Rc<RefCell<RigidBody>> {
        match *self {
            PlaneNode(ref n)             => n.body(),
            BallNode(ref n)              => n.body(),
            BoxNode(ref n)               => n.body(),
            CylinderNode(ref n)          => n.body(),
            ConeNode(ref n)              => n.body(),
            MeshNode(ref n)              => n.body(),
            BezierSurfaceNode(ref n)     => n.body(),
            ConvexNode(ref n)            => n.body()
        }
    }
}

pub struct GraphicsManager {
    rand:             XorShiftRng,
    rb2sn:            HashMap<uint, Vec<Node>>,
    rb2color:         HashMap<uint, Pnt3<f32>>,
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
        for _ in range(0u, 100) {
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

    pub fn remove(&mut self, window: &mut Window, body: &Rc<RefCell<RigidBody>>) {
        let key = body.deref() as *const RefCell<RigidBody> as uint;

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

    pub fn set_color(&mut self, body: &Rc<RefCell<RigidBody>>, color: Pnt3<f32>) {
        self.rb2color.insert(body.deref() as *const RefCell<RigidBody> as uint, color);
    }

    pub fn add(&mut self, window: &mut Window, body: Rc<RefCell<RigidBody>>) {
        let color;

        match self.rb2color.get(&(body.deref() as *const RefCell<RigidBody> as uint)) {
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
                          body:   Rc<RefCell<RigidBody>>,
                          color:  Pnt3<f32>) {
        let nodes = {
            let rb        = body.borrow();
            let mut nodes = Vec::new();

            self.add_geom(window, body.clone(), One::one(), rb.geom_ref(), color, &mut nodes);

            nodes
        };

        self.rb2sn.insert(body.deref() as *const RefCell<RigidBody> as uint, nodes);
    }

    fn add_geom(&mut self,
                window: &mut Window,
                body:   Rc<RefCell<RigidBody>>,
                delta:  Iso3<f32>,
                geom:   &Shape3,
                color:  Pnt3<f32>,
                out:    &mut Vec<Node>) {
        type Pl = shape::Plane3;
        type Bl = shape::Ball3;
        type Bo = shape::Cuboid3;
        type Cy = shape::Cylinder3;
        type Co = shape::Cone3;
        type Cm = shape::Compound3;
        type Tm = shape::Mesh3;
        type Bs = shape::BezierSurface3;
        type Cx = shape::Convex3;

        let id = geom.get_type_id();
        if id == TypeId::of::<Pl>(){
            self.add_plane(window, body, geom.downcast_ref::<Pl>().unwrap(), color, out)
        }
        else if id == TypeId::of::<Bl>() {
            self.add_ball(window, body, delta, geom.downcast_ref::<Bl>().unwrap(), color, out)
        }
        else if id == TypeId::of::<Bo>() {
            self.add_box(window, body, delta, geom.downcast_ref::<Bo>().unwrap(), color, out)
        }
        else if id == TypeId::of::<Cx>() {
            self.add_convex(window, body, delta, geom.downcast_ref::<Cx>().unwrap(), color, out)
        }
        else if id == TypeId::of::<Cy>() {
            self.add_cylinder(window, body, delta, geom.downcast_ref::<Cy>().unwrap(), color, out)
        }
        else if id == TypeId::of::<Co>() {
            self.add_cone(window, body, delta, geom.downcast_ref::<Co>().unwrap(), color, out)
        }
        else if id == TypeId::of::<Bs>() {
            self.add_bezier_surface(window, body, delta, geom.downcast_ref::<Bs>().unwrap(), color, out)
        }
        else if id == TypeId::of::<Cm>() {
            let c = geom.downcast_ref::<Cm>().unwrap();

            for &(t, ref s) in c.geoms().iter() {
                self.add_geom(window, body.clone(), delta * t, &***s, color, out)
            }
        }
        else if id == TypeId::of::<Tm>() {
            self.add_mesh(window, body, delta, geom.downcast_ref::<Tm>().unwrap(), color, out);
        }
        else {
            panic!("Not yet implemented.")
        }

    }

    fn add_plane(&mut self,
                 window: &mut Window,
                 body:   Rc<RefCell<RigidBody>>,
                 geom:   &shape::Plane3,
                 color:  Pnt3<f32>,
                 out:    &mut Vec<Node>) {
        let position = na::translation(&*body.borrow()).translate(&na::orig());
        let normal   = na::rotate(body.borrow().transform_ref(), geom.normal());

        out.push(PlaneNode(Plane::new(body, &position, &normal, color, window)))
    }

    fn add_mesh(&mut self,
                window: &mut Window,
                body:   Rc<RefCell<RigidBody>>,
                delta:  Iso3<f32>,
                geom:   &shape::Mesh3,
                color:  Pnt3<f32>,
                out:    &mut Vec<Node>) {
        let vertices = geom.vertices().deref();
        let indices  = geom.indices().deref();

        let vs     = vertices.clone();
        let mut is = Vec::new();

        for i in indices.as_slice().chunks(3) {
            is.push(Vec3::new(i[0] as u32, i[1] as u32, i[2] as u32))
        }

        out.push(MeshNode(Mesh::new(body, delta, vs, is, color, window)))
    }

    fn add_bezier_surface(&mut self,
                window: &mut Window,
                body:   Rc<RefCell<RigidBody>>,
                delta:  Iso3<f32>,
                geom:   &shape::BezierSurface3,
                color:  Pnt3<f32>,
                out:    &mut Vec<Node>) {
        out.push(BezierSurfaceNode(BezierSurface::new(body, delta, geom.control_points(), geom.nupoints(), geom.nvpoints(), color, window)))
    }

    fn add_ball(&mut self,
                window: &mut Window,
                body:   Rc<RefCell<RigidBody>>,
                delta:  Iso3<f32>,
                geom:   &shape::Ball3,
                color:  Pnt3<f32>,
                out:    &mut Vec<Node>) {
        let margin = body.borrow().margin();
        out.push(BallNode(Ball::new(body, delta, geom.radius() + margin, color, window)))
    }

    fn add_box(&mut self,
               window: &mut Window,
               body:   Rc<RefCell<RigidBody>>,
               delta:  Iso3<f32>,
               geom:   &shape::Cuboid3,
               color:  Pnt3<f32>,
               out:    &mut Vec<Node>) {
        let rx = geom.half_extents().x + body.borrow().margin();
        let ry = geom.half_extents().y + body.borrow().margin();
        let rz = geom.half_extents().z + body.borrow().margin();

        out.push(BoxNode(Box::new(body, delta, rx, ry, rz, color, window)))
    }

    fn add_convex(&mut self,
                  window: &mut Window,
                  body:   Rc<RefCell<RigidBody>>,
                  delta:  Iso3<f32>,
                  geom:   &shape::Convex3,
                  color:  Pnt3<f32>,
                  out:    &mut Vec<Node>) {
        out.push(ConvexNode(Convex::new(body, delta, &procedural::convex_hull3(geom.points()), color, window)))
    }

    fn add_cylinder(&mut self,
                    window: &mut Window,
                    body:   Rc<RefCell<RigidBody>>,
                    delta:  Iso3<f32>,
                    geom:   &shape::Cylinder3,
                    color:  Pnt3<f32>,
                    out:    &mut Vec<Node>) {
        let r = geom.radius();
        let h = geom.half_height() * 2.0;

        out.push(CylinderNode(Cylinder::new(body, delta, r, h, color, window)))
    }

    fn add_cone(&mut self,
                window: &mut Window,
                body:   Rc<RefCell<RigidBody>>,
                delta:  Iso3<f32>,
                geom:   &shape::Cone3,
                color:  Pnt3<f32>,
                out:    &mut Vec<Node>) {
        let r = geom.radius();
        let h = geom.half_height() * 2.0;

        out.push(ConeNode(Cone::new(body, delta, r, h, color, window)))
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

                let t      = na::transformation(rb.deref());
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
            self.first_person.look_at_z(self.arc_ball.eye(), self.arc_ball.at());
        }
        else {
            self.arc_ball.look_at_z(self.first_person.eye(), self.first_person.at());
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
        self.arc_ball.look_at_z(eye, at);
        self.first_person.look_at_z(eye, at);
    }

    pub fn body_to_scene_node(&mut self, rb: &Rc<RefCell<RigidBody>>) -> Option<&mut Vec<Node>> {
        self.rb2sn.get_mut(&(rb.deref() as *const RefCell<RigidBody> as uint))
    }
}
