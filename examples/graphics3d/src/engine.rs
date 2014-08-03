use std::intrinsics::TypeId;
use std::any::AnyRefExt;
use std::rc::Rc;
use std::cell::RefCell;
use std::num::One;
use std::collections::HashMap;
use rand::{SeedableRng, XorShiftRng, Rng};
use nalgebra::na::{Vec3, Iso3};
use nalgebra::na;
use kiss3d::loader::obj;
use kiss3d::window::Window;
use kiss3d::scene::SceneNode;
use kiss3d::camera::{Camera, ArcBall, FirstPerson};
use ncollide::utils::AnyPrivate;
use ncollide::geom::Geom;
use ncollide::geom;
use nphysics::world::World;
use nphysics::object::RigidBody;
use objects::bezier_surface::BezierSurface;
use objects::parametric_surface::ParametricSurface;
use objects::ball::Ball;
use objects::box_node::Box;
use objects::cylinder::Cylinder;
use objects::cone::Cone;
use objects::mesh::Mesh;
use objects::plane::Plane;
use simulate;

pub enum Node {
    BallNode(Ball),
    BoxNode(Box),
    CylinderNode(Cylinder),
    ConeNode(Cone),
    MeshNode(Mesh),
    PlaneNode(Plane),
    BezierSurfaceNode(BezierSurface),
    ParametricSurfaceNode(ParametricSurface)
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
            ParametricSurfaceNode(ref mut n) => n.select()
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
            ParametricSurfaceNode(ref mut n) => n.unselect()
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
            ParametricSurfaceNode(ref mut n) => n.update()
        }
    }

    pub fn object<'r>(&'r self) -> &'r SceneNode {
        match *self {
            PlaneNode(ref n)             => n.object(),
            BallNode(ref n)              => n.object(),
            BoxNode(ref n)               => n.object(),
            CylinderNode(ref n)          => n.object(),
            ConeNode(ref n)              => n.object(),
            MeshNode(ref n)              => n.object(),
            BezierSurfaceNode(ref n)     => n.object(),
            ParametricSurfaceNode(ref n) => n.object()
        }
    }

    pub fn object_mut<'r>(&'r mut self) -> &'r mut SceneNode {
        match *self {
            PlaneNode(ref mut n)             => n.object_mut(),
            BallNode(ref mut n)              => n.object_mut(),
            BoxNode(ref mut n)               => n.object_mut(),
            CylinderNode(ref mut n)          => n.object_mut(),
            ConeNode(ref mut n)              => n.object_mut(),
            MeshNode(ref mut n)              => n.object_mut(),
            BezierSurfaceNode(ref mut n)     => n.object_mut(),
            ParametricSurfaceNode(ref mut n) => n.object_mut()
        }
    }
}

pub struct GraphicsManager {
    rand:             XorShiftRng,
    rb2sn:            HashMap<uint, Vec<Node>>,
    arc_ball:         ArcBall,
    first_person:     FirstPerson,
    curr_is_arc_ball: bool

}

impl GraphicsManager {
    pub fn new() -> GraphicsManager {
        let arc_ball     = ArcBall::new(Vec3::new(10.0, 10.0, 10.0), Vec3::new(0.0, 0.0, 0.0));
        let first_person = FirstPerson::new(Vec3::new(10.0, 10.0, 10.0), Vec3::new(0.0, 0.0, 0.0));

        GraphicsManager {
            arc_ball:         arc_ball,
            first_person:     first_person,
            curr_is_arc_ball: true,
            rand:             SeedableRng::from_seed([0, 2, 4, 8]),
            rb2sn:            HashMap::new(),
        }
    }

    pub fn simulate(builder: |&mut Window, &mut GraphicsManager| -> World) {
        simulate::simulate(builder)
    }

    pub fn remove(&mut self, window: &mut Window, body: &Rc<RefCell<RigidBody>>) {
        let key = body.deref() as *const RefCell<RigidBody> as uint;

        match self.rb2sn.find(&key) {
            Some(sns) => {
                for sn in sns.iter() {
                    window.remove(&mut sn.object().clone());
                }
            }
            None => { }
        }

        self.rb2sn.remove(&key);
    }

    pub fn gen_color(&mut self) -> Vec3<f32> {
        self.rand.gen()
    }

    pub fn add(&mut self, window: &mut Window, body: Rc<RefCell<RigidBody>>) {
        let color = self.gen_color();
        self.add_with_color(window, body, color)
    }

    pub fn add_with_color(&mut self,
                          window: &mut Window,
                          body:   Rc<RefCell<RigidBody>>,
                          color:  Vec3<f32>) {
        let nodes = {
            let rb        = body.borrow();
            let mut nodes = Vec::new();

            self.add_geom(window, body.clone(), One::one(), rb.geom_ref(), color, &mut nodes);

            nodes
        };

        self.rb2sn.insert(body.deref() as *const RefCell<RigidBody> as uint, nodes);
    }

    pub fn load_mesh(&mut self, path: &str) -> Vec<(Vec<Vec3<f32>>, Vec<uint>)> {
        let path    = Path::new(path);
        let empty   = Path::new("_some_non_existant_folder"); // dont bother loading mtl files correctly
        let objects = obj::parse_file(&path, &empty, "").ok().expect("Unable to open the obj file.");

        let mut res = Vec::new();

        for (_, m, _) in objects.move_iter() {
            let vertices = m.coords().read().to_owned().unwrap();
            let indices  = m.faces().read().to_owned().unwrap();

            let mut flat_indices = Vec::new();

            for i in indices.move_iter() {
                flat_indices.push(i.x as uint);
                flat_indices.push(i.y as uint);
                flat_indices.push(i.z as uint);
            }

            let m = (vertices, flat_indices);

            res.push(m);
        }

        res
    }

    fn add_geom(&mut self,
                window: &mut Window,
                body:   Rc<RefCell<RigidBody>>,
                delta:  Iso3<f32>,
                geom:   &Geom,
                color:  Vec3<f32>,
                out:    &mut Vec<Node>) {
        type Pl = geom::Plane;
        type Bl = geom::Ball;
        type Bo = geom::Cuboid;
        type Cy = geom::Cylinder;
        type Co = geom::Cone;
        type Cm = geom::Compound;
        type Tm = geom::Mesh;
        type Bs = geom::BezierSurface;

        let id = geom.get_dyn_type_id();
        if id == TypeId::of::<Pl>(){
            self.add_plane(window, body, geom.as_ref::<Pl>().unwrap(), color, out)
        }
        else if id == TypeId::of::<Bl>() {
            self.add_ball(window, body, delta, geom.as_ref::<Bl>().unwrap(), color, out)
        }
        else if id == TypeId::of::<Bo>() {
            self.add_box(window, body, delta, geom.as_ref::<Bo>().unwrap(), color, out)
        }
        else if id == TypeId::of::<Cy>() {
            self.add_cylinder(window, body, delta, geom.as_ref::<Cy>().unwrap(), color, out)
        }
        else if id == TypeId::of::<Co>() {
            self.add_cone(window, body, delta, geom.as_ref::<Co>().unwrap(), color, out)
        }
        else if id == TypeId::of::<Bs>() {
            self.add_bezier_surface(window, body, delta, geom.as_ref::<Bs>().unwrap(), color, out)
        }
        else if id == TypeId::of::<Cm>() {
            let c = geom.as_ref::<Cm>().unwrap();

            for &(t, ref s) in c.geoms().iter() {
                self.add_geom(window, body.clone(), delta * t, **s, color, out)
            }
        }
        else if id == TypeId::of::<Tm>() {
            self.add_mesh(window, body, delta, geom.as_ref::<Tm>().unwrap(), color, out);
        }
        else {
            fail!("Not yet implemented.")
        }

    }

    fn add_plane(&mut self,
                 window: &mut Window,
                 body:   Rc<RefCell<RigidBody>>,
                 geom:   &geom::Plane,
                 color:  Vec3<f32>,
                 out:    &mut Vec<Node>) {
        let position = na::translation(body.borrow().deref());
        let normal   = na::rotate(body.borrow().transform_ref(), &geom.normal());

        out.push(PlaneNode(Plane::new(&position, &normal, color, window)))
    }

    fn add_mesh(&mut self,
                window: &mut Window,
                body:   Rc<RefCell<RigidBody>>,
                delta:  Iso3<f32>,
                geom:   &geom::Mesh,
                color:  Vec3<f32>,
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
                geom:   &geom::BezierSurface,
                color:  Vec3<f32>,
                out:    &mut Vec<Node>) {
        out.push(BezierSurfaceNode(BezierSurface::new(body, delta, geom.control_points(), geom.nupoints(), geom.nvpoints(), color, window)))
    }

    fn add_ball(&mut self,
                window: &mut Window,
                body:   Rc<RefCell<RigidBody>>,
                delta:  Iso3<f32>,
                geom:   &geom::Ball,
                color:  Vec3<f32>,
                out:    &mut Vec<Node>) {
        out.push(BallNode(Ball::new(body, delta, geom.radius(), color, window)))
    }

    fn add_box(&mut self,
               window: &mut Window,
               body:   Rc<RefCell<RigidBody>>,
               delta:  Iso3<f32>,
               geom:   &geom::Cuboid,
               color:  Vec3<f32>,
               out:    &mut Vec<Node>) {
        let rx = geom.half_extents().x + geom.margin();
        let ry = geom.half_extents().y + geom.margin();
        let rz = geom.half_extents().z + geom.margin();

        out.push(BoxNode(Box::new(body, delta, rx, ry, rz, color, window)))
    }

    fn add_cylinder(&mut self,
                    window: &mut Window,
                    body:   Rc<RefCell<RigidBody>>,
                    delta:  Iso3<f32>,
                    geom:   &geom::Cylinder,
                    color:  Vec3<f32>,
                    out:    &mut Vec<Node>) {
        let r = geom.radius();
        let h = geom.half_height() * 2.0;

        out.push(CylinderNode(Cylinder::new(body, delta, r, h, color, window)))
    }

    fn add_cone(&mut self,
                window: &mut Window,
                body:   Rc<RefCell<RigidBody>>,
                delta:  Iso3<f32>,
                geom:   &geom::Cone,
                color:  Vec3<f32>,
                out:    &mut Vec<Node>) {
        let r = geom.radius();
        let h = geom.half_height() * 2.0;

        out.push(ConeNode(Cone::new(body, delta, r, h, color, window)))
    }

    pub fn draw(&mut self) {
        for (_, ns) in self.rb2sn.mut_iter() {
            for n in ns.mut_iter() {
                n.update()
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

    pub fn look_at(&mut self, eye: Vec3<f32>, at: Vec3<f32>) {
        self.arc_ball.look_at_z(eye, at);
        self.first_person.look_at_z(eye, at);
    }

    pub fn body_to_scene_node<'r>(&'r mut self, rb: &Rc<RefCell<RigidBody>>) -> Option<&'r mut Vec<Node>> {
        self.rb2sn.find_mut(&(rb.deref() as *const RefCell<RigidBody> as uint))
    }
}
