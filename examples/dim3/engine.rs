use std::ptr;
use std::rand::{XorShiftRng, RngUtil};
use std::hashmap::HashMap;
use nalgebra::vec::Vec3;
use nalgebra::traits::rotation::Rotate;
use nalgebra::traits::translation::Translation;
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
            rb2sn:   HashMap::new(),
            obj2color: HashMap::new()
        }
    }

    pub fn simulate(builder: ~fn(&mut GraphicsManager) -> (dim3::World3d<f64>,
                                                           @mut dim3::DBVTCollisionDetector3d<f64>,
                                                           @mut dim3::DBVTSweptBallMotionClamping3d<f64>)) {
        simulate::simulate(builder)
    }

    pub fn add_plane(&mut self,
                     body:  @mut dim3::RigidBody3d<f64>,
                     geom:  &dim3::Plane3d<f64>) {
        let obj = @mut Plane::new(body,
                                  &body.transform_ref().translation(),
                                  &body.transform_ref().rotate(&geom.normal()),
                                  self.color_for_object(body),
                                  self.window);
        do self.rb2sn.insert_or_update_with(ptr::to_mut_unsafe_ptr(body) as uint, ~[obj as @mut SceneNode]) |_, v| {
            v.push(obj as @mut SceneNode)
        };
    }

    pub fn add_ball(&mut self,
                    body:  @mut dim3::RigidBody3d<f64>,
                    delta: dim3::Transform3d<f64>,
                    geom:  &dim3::Ball3d<f64>) {
        let obj = @mut Ball::new(body, delta, geom.radius(), self.color_for_object(body), self.window);
        do self.rb2sn.insert_or_update_with(ptr::to_mut_unsafe_ptr(body) as uint, ~[obj as @mut SceneNode]) |_, v| {
            v.push(obj as @mut SceneNode)
        };
    }

    pub fn add_cube(&mut self,
                    body:  @mut dim3::RigidBody3d<f64>,
                    delta: dim3::Transform3d<f64>,
                    geom:  &dim3::Box3d<f64>) {
        let rx = geom.half_extents().x;
        let ry = geom.half_extents().y;
        let rz = geom.half_extents().z;

        let obj = @mut Box::new(body, delta, rx, ry, rz, self.color_for_object(body), self.window);
        do self.rb2sn.insert_or_update_with(ptr::to_mut_unsafe_ptr(body) as uint, ~[obj as @mut SceneNode]) |_, v| {
            v.push(obj as @mut SceneNode)
        };
    }

    pub fn add_cylinder(&mut self,
                        body:  @mut dim3::RigidBody3d<f64>,
                        delta: dim3::Transform3d<f64>,
                        geom:  &dim3::Cylinder3d<f64>) {
        let r = geom.radius();
        let h = geom.half_height() * 2.0;

        let obj = @mut Cylinder::new(body, delta, r, h, self.color_for_object(body), self.window);
        do self.rb2sn.insert_or_update_with(ptr::to_mut_unsafe_ptr(body) as uint, ~[obj as @mut SceneNode]) |_, v| {
            v.push(obj as @mut SceneNode)
        };
    }

    pub fn add_cone(&mut self,
                    body:  @mut dim3::RigidBody3d<f64>,
                    delta: dim3::Transform3d<f64>,
                    geom:  &dim3::Cone3d<f64>) {
        let r = geom.radius();
        let h = geom.half_height() * 2.0;

        let obj = @mut Cone::new(body, delta, r, h, self.color_for_object(body), self.window);
        do self.rb2sn.insert_or_update_with(ptr::to_mut_unsafe_ptr(body) as uint, ~[obj as @mut SceneNode]) |_, v| {
            v.push(obj as @mut SceneNode)
        };
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

    pub fn rigid_body_to_scene_node<'r>(&'r self, rb: @mut dim3::RigidBody3d<f64>) -> Option<&'r ~[@mut SceneNode]> {
        self.rb2sn.find(&(ptr::to_mut_unsafe_ptr(rb) as uint))
    }

    pub fn color_for_object(&mut self, body: @mut dim3::RigidBody3d<f64>) -> Vec3<f32> {
        let key = ptr::to_mut_unsafe_ptr(body) as uint;
        match self.obj2color.find(&key) {
            Some(color) => return *color,
            None => { }
        }

        let color = self.rand.gen();

        self.obj2color.insert(key, color);

        color
    }
}
