use std::unstable::intrinsics::TypeId;
use std::rc::Rc;
use std::cell::RefCell;
use std::num::One;
use std::rand::{SeedableRng, XorShiftRng, Rng};
use std::hashmap::HashMap;
use nalgebra::na::{Vec3, Iso3};
use nalgebra::na;
use kiss3d::loader::obj;
use kiss3d::window::Window;
use kiss3d::object::Object;
use kiss3d::camera::{Camera, ArcBall, FirstPerson};
use ncollide::geom::Geom;
use ncollide::geom;
use nphysics::world::World;
use nphysics::object::RigidBody;
use objects::ball::Ball;
use objects::box_node::Box;
use objects::cylinder::Cylinder;
use objects::cone::Cone;
use objects::mesh::Mesh;
use objects::plane::Plane;
use simulate;

pub enum SceneNode {
    BallNode(Ball),
    BoxNode(Box),
    CylinderNode(Cylinder),
    ConeNode(Cone),
    MeshNode(Mesh),
    PlaneNode(Plane)
}

impl SceneNode {
    pub fn select(&mut self) {
        match *self {
            PlaneNode(ref mut n)    => n.select(),
            BallNode(ref mut n)     => n.select(),
            BoxNode(ref mut n)      => n.select(),
            CylinderNode(ref mut n) => n.select(),
            ConeNode(ref mut n)     => n.select(),
            MeshNode(ref mut n)     => n.select()
        }
    }

    pub fn unselect(&mut self) {
        match *self {
            PlaneNode(ref mut n)    => n.unselect(),
            BallNode(ref mut n)     => n.unselect(),
            BoxNode(ref mut n)      => n.unselect(),
            CylinderNode(ref mut n) => n.unselect(),
            ConeNode(ref mut n)     => n.unselect(),
            MeshNode(ref mut n)     => n.unselect()
        }
    }

    pub fn update(&mut self) {
        match *self {
            PlaneNode(ref mut n)    => n.update(),
            BallNode(ref mut n)     => n.update(),
            BoxNode(ref mut n)      => n.update(),
            CylinderNode(ref mut n) => n.update(),
            ConeNode(ref mut n)     => n.update(),
            MeshNode(ref mut n)     => n.update()
        }
    }

    pub fn object<'r>(&'r self) -> &'r Object {
        match *self {
            PlaneNode(ref n)    => n.object(),
            BallNode(ref n)     => n.object(),
            BoxNode(ref n)      => n.object(),
            CylinderNode(ref n) => n.object(),
            ConeNode(ref n)     => n.object(),
            MeshNode(ref n)     => n.object()
        }
    }
}

pub struct GraphicsManager {
    rand:             XorShiftRng,
    rb2sn:            HashMap<uint, ~[SceneNode]>,
    obj2color:        HashMap<uint, Vec3<f32>>,
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
            obj2color:        HashMap::new()
        }
    }

    pub fn init_camera<'a>(&'a mut self, window: &'a mut Window) {
        window.set_camera(&'a mut self.arc_ball as &'a mut Camera);
    }

    pub fn simulate(builder: proc(&mut Window, &mut GraphicsManager) -> World) {
        simulate::simulate(builder)
    }

    pub fn remove(&mut self, window: &mut Window, body: &Rc<RefCell<RigidBody>>) {
        let key = body.borrow() as *RefCell<RigidBody> as uint;

        match self.rb2sn.find(&key) {
            Some(sns) => {
                for sn in sns.iter() {
                    window.remove(sn.object().clone());
                }
            }
            None => { }
        }

        self.rb2sn.remove(&key);
        self.obj2color.remove(&key);
    }

    pub fn add(&mut self, window: &mut Window, body: Rc<RefCell<RigidBody>>) {

        let nodes = {
            let rb        = body.borrow().borrow();
            let mut nodes = ~[];

            self.add_geom(window, body.clone(), One::one(), rb.get().geom(), &mut nodes);

            nodes
        };

        self.rb2sn.insert(body.borrow() as *RefCell<RigidBody> as uint, nodes);
    }

    pub fn load_mesh(&mut self, path: &str) -> ~[(~[Vec3<f32>], ~[uint])] {
        let path    = Path::new(path);
        let empty   = Path::new("_some_non_existant_folder"); // dont bother loading mtl files correctly
        let objects = obj::parse_file(&path, &empty, "").expect("Unable to open the obj file.");

        let mut res = ~[];

        for (_, m, _) in objects.move_iter() {
            let vertices = m.coords().read(|v| v.to_owned()).unwrap();
            let indices  = m.faces().read(|v| v.to_owned()).unwrap();

            let m = (vertices, indices.flat_map(|i| ~[i.x as uint, i.y as uint, i.z as uint]));

            res.push(m);
        }

        res
    }

    fn add_geom(&mut self,
                window: &mut Window,
                body:   Rc<RefCell<RigidBody>>,
                delta:  Iso3<f32>,
                geom:   &Geom,
                out:    &mut ~[SceneNode]) {
        type Pl = geom::Plane;
        type Bl = geom::Ball;
        type Bo = geom::Box;
        type Cy = geom::Cylinder;
        type Co = geom::Cone;
        type Cm = geom::Compound;
        type Tm = geom::Mesh;

        let id = geom.get_type_id();
        if id == TypeId::of::<Pl>(){
            self.add_plane(window, body, geom.as_ref::<Pl>().unwrap(), out)
        }
        else if id == TypeId::of::<Bl>() {
            self.add_ball(window, body, delta, geom.as_ref::<Bl>().unwrap(), out)
        }
        else if id == TypeId::of::<Bo>() {
            self.add_box(window, body, delta, geom.as_ref::<Bo>().unwrap(), out)
        }
        else if id == TypeId::of::<Cy>() {
            self.add_cylinder(window, body, delta, geom.as_ref::<Cy>().unwrap(), out)
        }
        else if id == TypeId::of::<Co>() {
            self.add_cone(window, body, delta, geom.as_ref::<Co>().unwrap(), out)
        }
        else if id == TypeId::of::<Cm>() {
            let c = geom.as_ref::<Cm>().unwrap();

            for &(t, ref s) in c.shapes().iter() {
                self.add_geom(window, body.clone(), delta * t, *s, out)
            }
        }
        else if id == TypeId::of::<Tm>() {
            self.add_mesh(window, body, delta, geom.as_ref::<Tm>().unwrap(), out);
        }
        else {
            fail!("Not yet implemented.")
        }

    }

    fn add_plane(&mut self,
                 window: &mut Window,
                 body:   Rc<RefCell<RigidBody>>,
                 geom:   &geom::Plane,
                 out:    &mut ~[SceneNode]) {
        let position = body.borrow().with(|body| na::translation(body));
        let normal   = body.borrow().with(|body| na::rotate(body.transform_ref(), &geom.normal()));
        let color    = self.color_for_object(&body);

        out.push(PlaneNode(Plane::new(body, &position, &normal, color, window)))
    }

    fn add_mesh(&mut self,
                window: &mut Window,
                body:   Rc<RefCell<RigidBody>>,
                delta:  Iso3<f32>,
                geom:   &geom::Mesh,
                out:    &mut ~[SceneNode]) {
        let color    = self.color_for_object(&body);
        let vertices = geom.vertices().get();
        let indices  = geom.indices().get();

        let vs     = vertices.to_owned();
        let mut is = ~[];

        for i in indices.chunks(3) {
            is.push(Vec3::new(i[0] as u32, i[1] as u32, i[2] as u32))
        }

        out.push(MeshNode(Mesh::new(body, delta, vs, is, color, window)))
    }

    fn add_ball(&mut self,
                window: &mut Window,
                body:   Rc<RefCell<RigidBody>>,
                delta:  Iso3<f32>,
                geom:   &geom::Ball,
                out:    &mut ~[SceneNode]) {
        let color = self.color_for_object(&body);
        out.push(BallNode(Ball::new(body, delta, geom.radius(), color, window)))
    }

    fn add_box(&mut self,
               window: &mut Window,
               body:   Rc<RefCell<RigidBody>>,
               delta:  Iso3<f32>,
               geom:   &geom::Box,
               out:    &mut ~[SceneNode]) {
        let rx = geom.half_extents().x + geom.margin();
        let ry = geom.half_extents().y + geom.margin();
        let rz = geom.half_extents().z + geom.margin();

        let color = self.color_for_object(&body);

        out.push(BoxNode(Box::new(body, delta, rx, ry, rz, color, window)))
    }

    fn add_cylinder(&mut self,
                    window: &mut Window,
                    body:   Rc<RefCell<RigidBody>>,
                    delta:  Iso3<f32>,
                    geom:   &geom::Cylinder,
                    out:    &mut ~[SceneNode]) {
        let r = geom.radius();
        let h = geom.half_height() * 2.0;

        let color = self.color_for_object(&body);

        out.push(CylinderNode(Cylinder::new(body, delta, r, h, color, window)))
    }

    fn add_cone(&mut self,
                window: &mut Window,
                body:   Rc<RefCell<RigidBody>>,
                delta:  Iso3<f32>,
                geom:   &geom::Cone,
                out:    &mut ~[SceneNode]) {
        let r = geom.radius();
        let h = geom.half_height() * 2.0;

        let color = self.color_for_object(&body);

        out.push(ConeNode(Cone::new(body, delta, r, h, color, window)))
    }

    pub fn draw(&mut self) {
        for (_, ns) in self.rb2sn.mut_iter() {
            for n in ns.mut_iter() {
                n.update()
            }
        }
    }

    pub fn switch_cameras<'a>(&'a mut self, window: &'a mut Window) {
        if self.curr_is_arc_ball {
            self.first_person.look_at_z(self.arc_ball.eye(), self.arc_ball.at());
            window.set_camera(&'a mut self.first_person as &'a mut Camera);
        }
        else {
            self.arc_ball.look_at_z(self.first_person.eye(), self.first_person.at());
            window.set_camera(&'a mut self.arc_ball as &'a mut Camera);
        }

        self.curr_is_arc_ball = !self.curr_is_arc_ball;
    }

    pub fn look_at(&mut self, eye: Vec3<f32>, at: Vec3<f32>) {
        self.arc_ball.look_at_z(eye, at);
        self.first_person.look_at_z(eye, at);
    }

    pub fn body_to_scene_node<'r>(&'r mut self, rb: &Rc<RefCell<RigidBody>>) -> Option<&'r mut ~[SceneNode]> {
        self.rb2sn.find_mut(&(rb.borrow() as *RefCell<RigidBody> as uint))
    }

    pub fn color_for_object(&mut self, body: &Rc<RefCell<RigidBody>>) -> Vec3<f32> {
        let key = body.borrow() as *RefCell<RigidBody> as uint;
        match self.obj2color.find(&key) {
            Some(color) => return *color,
            None => { }
        }

        let color = self.rand.gen();

        self.obj2color.insert(key, color);

        color
    }
}
