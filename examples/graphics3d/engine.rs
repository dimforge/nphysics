use std::num::One;
use std::ptr;
use std::rand::{SeedableRng, XorShiftRng, Rng};
use std::hashmap::HashMap;
use nalgebra::na::Vec3;
use nalgebra::na;
use ncollide::geom::{PlaneGeom, ImplicitGeom, CompoundGeom, BallGeom, BoxGeom, CylinderGeom, CapsuleGeom, ConeGeom};
use kiss3d::window::Window;
use kiss3d::object::Object;
use kiss3d::camera::{Camera, ArcBall, FirstPerson};
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
    fn object<'r>(&'r self) -> &'r Object;
}

pub struct GraphicsManager {
    rand:             XorShiftRng,
    rb2sn:            HashMap<uint, ~[@mut SceneNode]>,
    obj2color:        HashMap<uint, Vec3<f32>>,
    arc_ball:         @mut ArcBall,
    first_person:     @mut FirstPerson,
    curr_is_arc_ball: bool

}

impl GraphicsManager {
    pub fn new(window: &mut Window) -> GraphicsManager {
        let arc_ball     = @mut ArcBall::new(Vec3::new(10.0, 10.0, 10.0), Vec3::new(0.0, 0.0, 0.0));
        let first_person = @mut FirstPerson::new(Vec3::new(10.0, 10.0, 10.0), Vec3::new(0.0, 0.0, 0.0));

        window.set_camera(arc_ball as @mut Camera);

        GraphicsManager {
            arc_ball:         arc_ball,
            first_person:     first_person,
            curr_is_arc_ball: true,
            rand:             SeedableRng::from_seed([0, 2, 4, 8]),
            rb2sn:            HashMap::new(),
            obj2color:        HashMap::new()
        }
    }

    pub fn simulate(builder: proc(&mut Window, &mut GraphicsManager) -> dim3::BodyWorld3d<f32>) {
        simulate::simulate(builder)
    }

    pub fn remove(&mut self, window: &mut Window, body: @mut dim3::Body3d<f32>) {
        let key = ptr::to_mut_unsafe_ptr(body) as uint;

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

    pub fn add(&mut self, window: &mut Window, body: @mut dim3::Body3d<f32>) {

        let nodes = {
            let rb = body.to_rigid_body_or_fail();
            let mut nodes = ~[];

            self.add_geom(window, body, One::one(), rb.geom(), &mut nodes);

            nodes
        };

        self.rb2sn.insert(ptr::to_mut_unsafe_ptr(body) as uint, nodes);
    }

    fn add_geom(&mut self,
                window: &mut Window,
                body:   @mut dim3::Body3d<f32>,
                delta:  dim3::Transform3d<f32>,
                geom:   &dim3::Geom3d<f32>,
                out:    &mut ~[@mut SceneNode]) {
        match *geom {
            PlaneGeom(ref p)    => self.add_plane(window, body, p, out),
            CompoundGeom(ref c) => {
                for &(t, ref s) in c.get().shapes().iter() {
                    self.add_geom(window, body, delta * t, s, out)
                }
            },
            ImplicitGeom(ref i) => {
                match *i {
                    BallGeom(ref b)     => self.add_ball(window, body, delta, b, out),
                    BoxGeom(ref b)      => self.add_box(window, body, delta, b, out),
                    CylinderGeom(ref c) => self.add_cylinder(window, body, delta, c, out),
                    ConeGeom(ref c)     => self.add_cone(window, body, delta, c, out),
                    CapsuleGeom(_)      => fail!("Not yet implemented.")
                }
            },
        }
    }

    fn add_plane(&mut self,
                 window: &mut Window,
                 body:   @mut dim3::Body3d<f32>,
                 geom:   &dim3::Plane3d<f32>,
                 out:    &mut ~[@mut SceneNode]) {
        let position = na::translation(body.to_rigid_body_or_fail());
        let normal   = na::rotate(body.to_rigid_body_or_fail().transform_ref(), &geom.normal());
        let color    = self.color_for_object(body);

        out.push(@mut Plane::new(body, &position, &normal, color, window) as @mut SceneNode)
    }

    fn add_ball(&mut self,
                window: &mut Window,
                body:   @mut dim3::Body3d<f32>,
                delta:  dim3::Transform3d<f32>,
                geom:   &dim3::Ball3d<f32>,
                out:    &mut ~[@mut SceneNode]) {
        let color = self.color_for_object(body);
        out.push(@mut Ball::new(body, delta, geom.radius(), color, window) as @mut SceneNode)
    }

    fn add_box(&mut self,
               window: &mut Window,
               body:   @mut dim3::Body3d<f32>,
               delta:  dim3::Transform3d<f32>,
               geom:   &dim3::Box3d<f32>,
               out:    &mut ~[@mut SceneNode]) {
        let rx = geom.half_extents().x;
        let ry = geom.half_extents().y;
        let rz = geom.half_extents().z;

        let color = self.color_for_object(body);

        out.push(@mut Box::new(body, delta, rx, ry, rz, color, window) as @mut SceneNode)
    }

    fn add_cylinder(&mut self,
                    window: &mut Window,
                    body:   @mut dim3::Body3d<f32>,
                    delta:  dim3::Transform3d<f32>,
                    geom:   &dim3::Cylinder3d<f32>,
                    out:    &mut ~[@mut SceneNode]) {
        let r = geom.radius();
        let h = geom.half_height() * 2.0;

        let color = self.color_for_object(body);

        out.push(@mut Cylinder::new(body, delta, r, h, color, window) as @mut SceneNode)
    }

    fn add_cone(&mut self,
                window: &mut Window,
                body:   @mut dim3::Body3d<f32>,
                delta:  dim3::Transform3d<f32>,
                geom:   &dim3::Cone3d<f32>,
                out:    &mut ~[@mut SceneNode]) {
        let r = geom.radius();
        let h = geom.half_height() * 2.0;

        let color = self.color_for_object(body);

        out.push(@mut Cone::new(body, delta, r, h, color, window) as @mut SceneNode)
    }

    pub fn draw(&mut self) {
        for (_, ns) in self.rb2sn.mut_iter() {
            for n in ns.iter() {
                n.update()
            }
        }
    }

    pub fn switch_cameras(&mut self, window: &mut Window) {
        if self.curr_is_arc_ball {
            self.first_person.look_at_z(self.arc_ball.eye(), self.arc_ball.at());
            window.set_camera(self.first_person as @mut Camera);
        }
        else {
            self.arc_ball.look_at_z(self.first_person.eye(), self.first_person.at());
            window.set_camera(self.arc_ball as @mut Camera);
        }

        self.curr_is_arc_ball = !self.curr_is_arc_ball;
    }

    pub fn look_at(&mut self, eye: Vec3<f32>, at: Vec3<f32>) {
        self.arc_ball.look_at_z(eye, at);
        self.first_person.look_at_z(eye, at);
    }

    pub fn body_to_scene_node<'r>(&'r self, rb: @mut dim3::Body3d<f32>) -> Option<&'r ~[@mut SceneNode]> {
        self.rb2sn.find(&(ptr::to_mut_unsafe_ptr(rb) as uint))
    }

    pub fn color_for_object(&mut self, body: &dim3::Body3d<f32>) -> Vec3<f32> {
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
