use std::ptr;
use std::rand::{XorShiftRng, RngUtil};
use std::hashmap::HashMap;
use rsfml::graphics::render_window::RenderWindow;
use nalgebra::vec::Vec3;
use nphysics::aliases::dim2;
use camera::Camera;
use objects::ball::Ball;
use objects::box::Box;
use simulate;

pub trait SceneNode {
    fn update(&mut self);
    fn draw(&self, &mut RenderWindow);
}

pub struct GraphicsManager {
    rand:      XorShiftRng,
    objects:   ~[@mut SceneNode],
    obj2color: HashMap<uint, Vec3<u8>>

}

impl GraphicsManager {
    pub fn new() -> GraphicsManager {
        GraphicsManager {
            rand:      XorShiftRng::new_seeded(0, 1, 2, 3),
            objects:   ~[],
            obj2color: HashMap::new()
        }
    }

    pub fn simulate(builder: &fn(&mut GraphicsManager) -> dim2::World2d<f64>) {
        simulate::simulate(builder)
    }

    pub fn add_plane(&mut self, _: @mut dim2::RigidBody2d<f64>, _: &dim2::Plane2d<f64>) {
        // we dont draw anything for the plane atm
    }

    pub fn add_ball(&mut self,
                    body:  @mut dim2::RigidBody2d<f64>,
                    delta: dim2::Transform2d<f64>,
                    geom:  &dim2::Ball2d<f64>) {
        let color = self.color_for_object(body);

        self.objects.push(
            @mut Ball::new(body, delta, geom.radius(), color) as @mut SceneNode
        );
    }

    pub fn add_cube(&mut self,
                    body: @mut dim2::RigidBody2d<f64>,
                    delta: dim2::Transform2d<f64>,
                    geom: &dim2::Box2d<f64>) {
        let rx = geom.half_extents().x;
        let ry = geom.half_extents().y;

        let color = self.color_for_object(body);

        self.objects.push(
            @mut Box::new(body, delta, rx, ry, color) as @mut SceneNode
        );
    }

    pub fn draw(&mut self, rw: &mut RenderWindow, c: &Camera) {
        c.activate_scene(rw);

        for n in self.objects.mut_iter() {
            n.update();
        }

        for n in self.objects.mut_iter() {
            n.draw(rw);
        }

        c.activate_ui(rw);
    }

    pub fn color_for_object(&mut self, body: @mut dim2::RigidBody2d<f64>) -> Vec3<u8> {
        let key = ptr::to_mut_unsafe_ptr(body) as uint;
        match self.obj2color.find(&key) {
            Some(color) => return *color,
            None => { }
        }

        let color = Vec3::new(
            self.rand.gen_uint_range(0, 256) as u8,
            self.rand.gen_uint_range(0, 256) as u8,
            self.rand.gen_uint_range(0, 256) as u8);

        self.obj2color.insert(key, color);

        color
    }
}
