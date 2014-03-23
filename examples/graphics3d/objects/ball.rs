use std::rc::Rc;
use std::cell::RefCell;
use kiss3d::window::Window;
use kiss3d::object::Object;
use nalgebra::na::{Vec3, Iso3, Transformation};
use nalgebra::na;
use nphysics::object::RigidBody;

pub struct Ball {
    priv color:      Vec3<f32>,
    priv base_color: Vec3<f32>,
    priv delta:      Iso3<f32>,
    priv gfx:        Object,
    priv body:       Rc<RefCell<RigidBody>>
}

impl Ball {
    pub fn new(body:   Rc<RefCell<RigidBody>>,
               delta:  Iso3<f32>,
               radius: f32,
               color:  Vec3<f32>,
               window: &mut Window) -> Ball {
        let mut res = Ball {
            color:      color,
            base_color: color,
            delta:      delta,
            gfx:        window.add_sphere(radius as f32),
            body:       body
        };

        res.gfx.set_color(color.x, color.y, color.z);
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
            {
                self.gfx.set_transformation(na::transformation(rb.deref()) * self.delta);
            }

            self.gfx.set_color(self.color.x, self.color.y, self.color.z);
        }
        else {
            self.gfx.set_color(self.color.x * 0.25, self.color.y * 0.25, self.color.z * 0.25);
        }
    }

    pub fn object<'r>(&'r self) -> &'r Object {
        &'r self.gfx
    }
}
