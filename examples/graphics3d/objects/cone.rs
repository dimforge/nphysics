use std::rc::Rc;
use std::cell::RefCell;
use kiss3d::window;
use kiss3d::scene::SceneNode;
use nalgebra::na::{Vec3, Iso3};
use nalgebra::na;
use nphysics::object::RigidBody;

pub struct Cone {
    color:      Vec3<f32>,
    base_color: Vec3<f32>,
    delta:      Iso3<f32>,
    gfx:        SceneNode,
    body:       Rc<RefCell<RigidBody>>,
}

impl Cone {
    pub fn new(body:   Rc<RefCell<RigidBody>>,
               delta:  Iso3<f32>,
               r:      f32,
               h:      f32,
               color:  Vec3<f32>,
               window: &mut window::Window) -> Cone {
        let _frac_pi_2: f32 = Float::frac_pi_2();
        let t = na::transformation(body.borrow().deref());

        let mut res = Cone {
            color:      color,
            base_color: color,
            delta:      delta,
            gfx:        window.add_cone(r as f32, h as f32),
            body:       body
        };

        res.gfx.set_color(color.x, color.y, color.z);
        res.gfx.set_local_transformation(t * res.delta);
        res.update();

        res
    }

    pub fn select(&mut self) {
        self.color = Vec3::x();
    }

    pub fn unselect(&mut self) {
        self.color = self.base_color;
    }

    pub fn update(&mut self) {
        let rb = self.body.borrow();

        if rb.is_active() {
            self.gfx.set_local_transformation(na::transformation(rb.deref()) * self.delta);
            self.gfx.set_color(self.color.x, self.color.y, self.color.z);
        }
        else {
            self.gfx.set_color(self.color.x * 0.25, self.color.y * 0.25, self.color.z * 0.25);
        }
    }

    pub fn object<'r>(&'r self) -> &'r SceneNode {
        &self.gfx
    }

    pub fn object_mut<'r>(&'r mut self) -> &'r mut SceneNode {
        &'r mut self.gfx
    }
}
