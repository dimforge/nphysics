use std::num::One;
use std::ptr;
use std::rand::{XorShiftRng, RngUtil};
use std::hashmap::HashMap;
use nalgebra::vec::Vec3;
use nalgebra::traits::rotation::Rotate;
use nalgebra::traits::translation::Translation;
use ncollide::geom::{PlaneGeom, ImplicitGeom, CompoundGeom, BallGeom, BoxGeom, CylinderGeom, ConeGeom};
use kiss3d::window::Window;
use nphysics::aliases::dim3;
use objects::ball::Ball;
use objects::box::Box;
use objects::cylinder::Cylinder;
use objects::cone::Cone;
use objects::plane::Plane;
use simulate;

pub trait SceneNode {
    fn select(&mut self);
    fn unselect(&mut self);
    fn update(&mut self);
}

pub struct GraphicsManager {
    window:    @mut Window,
    rand:      XorShiftRng,
    rb2sn:     HashMap<uint, ~[@mut SceneNode]>,
    obj2color: HashMap<uint, Vec3<f32>>
}

impl GraphicsManager {
    pub fn new(window: @mut Window) -> GraphicsManager {
        GraphicsManager {
            window:    window,
            rand:      XorShiftRng::new_seeded(0, 1, 2, 3),
            rb2sn:     HashMap::new(),
            obj2color: HashMap::new()
        }
    }

    pub fn simulate(builder: ~fn(&mut GraphicsManager) -> dim3::BodyWorld3d<f64>) {
        simulate::simulate(builder)
    }

    pub fn add(&mut self, body: @mut dim3::Body3d<f64>) {

        let nodes = {
            let rb = body.to_rigid_body_or_fail();
            let mut nodes = ~[];

            self.add_geom(body, One::one(), rb.geom(), &mut nodes);

            nodes
        };

        self.rb2sn.insert(ptr::to_mut_unsafe_ptr(body) as uint, nodes);
    }

    fn add_geom(&mut self,
                body:  @mut dim3::Body3d<f64>,
                delta: dim3::Transform3d<f64>,
                geom:  &dim3::Geom3d<f64>,
                out:   &mut ~[@mut SceneNode]) {
        match *geom {
            PlaneGeom(ref p)    => self.add_plane(body, p, out),
            CompoundGeom(ref c) => {
                for &(t, ref s) in c.shapes().iter() {
                    self.add_geom(body, delta * t, s, out)
                }
            },
            ImplicitGeom(ref i) => {
                match *i {
                    BallGeom(ref b)     => self.add_ball(body, delta, b, out),
                    BoxGeom(ref b)      => self.add_box(body, delta, b, out),
                    CylinderGeom(ref c) => self.add_cylinder(body, delta, c, out),
                    ConeGeom(ref c)     => self.add_cone(body, delta, c, out),
                }
            },
        }
    }

    fn add_plane(&mut self,
                 body: @mut dim3::Body3d<f64>,
                 geom: &dim3::Plane3d<f64>,
                 out:  &mut ~[@mut SceneNode]) {
        let position = body.to_rigid_body_or_fail().translation();
        let normal   = body.to_rigid_body_or_fail().transform_ref().rotate(&geom.normal());
        let color    = self.color_for_object(body);

        out.push(@mut Plane::new(body, &position, &normal, color, self.window) as @mut SceneNode)
    }

    fn add_ball(&mut self,
                body:  @mut dim3::Body3d<f64>,
                delta: dim3::Transform3d<f64>,
                geom:  &dim3::Ball3d<f64>,
                out:   &mut ~[@mut SceneNode]) {
        let color = self.color_for_object(body);
        out.push(@mut Ball::new(body, delta, geom.radius(), color, self.window) as @mut SceneNode)
    }

    fn add_box(&mut self,
               body:  @mut dim3::Body3d<f64>,
               delta: dim3::Transform3d<f64>,
               geom:  &dim3::Box3d<f64>,
               out:   &mut ~[@mut SceneNode]) {
        let rx = geom.half_extents().x;
        let ry = geom.half_extents().y;
        let rz = geom.half_extents().z;

        let color = self.color_for_object(body);

        out.push(@mut Box::new(body, delta, rx, ry, rz, color, self.window) as @mut SceneNode)
    }

    fn add_cylinder(&mut self,
                    body:  @mut dim3::Body3d<f64>,
                    delta: dim3::Transform3d<f64>,
                    geom:  &dim3::Cylinder3d<f64>,
                    out:   &mut ~[@mut SceneNode]) {
        let r = geom.radius();
        let h = geom.half_height() * 2.0;

        let color = self.color_for_object(body);

        out.push(@mut Cylinder::new(body, delta, r, h, color, self.window) as @mut SceneNode)
    }

    fn add_cone(&mut self,
                body:  @mut dim3::Body3d<f64>,
                delta: dim3::Transform3d<f64>,
                geom:  &dim3::Cone3d<f64>,
                out:   &mut ~[@mut SceneNode]) {
        let r = geom.radius();
        let h = geom.half_height() * 2.0;

        let color = self.color_for_object(body);

        out.push(@mut Cone::new(body, delta, r, h, color, self.window) as @mut SceneNode)
    }

    pub fn draw(&mut self) {
        for (_, ns) in self.rb2sn.mut_iter() {
            for n in ns.iter() {
                n.update()
            }
        }
    }

    pub fn look_at(&mut self, eye: Vec3<f64>, at: Vec3<f64>) {
        self.window.camera().look_at_z(eye, at);
    }

    pub fn body_to_scene_node<'r>(&'r self, rb: @mut dim3::Body3d<f64>) -> Option<&'r ~[@mut SceneNode]> {
        self.rb2sn.find(&(ptr::to_mut_unsafe_ptr(rb) as uint))
    }

    pub fn color_for_object(&mut self, body: &dim3::Body3d<f64>) -> Vec3<f32> {
        let key = ptr::to_unsafe_ptr(body) as uint;
        match self.obj2color.find(&key) {
            Some(color) => return *color,
            None => { }
        }

        let color = self.rand.gen();

        self.obj2color.insert(key, color);

        color
    }
}
