use std::rc::Rc;
use std::cell::RefCell;
use kiss3d::window;
use kiss3d::object::Object;
use nalgebra::na::Vec3;
use nphysics::object::RigidBody;

pub struct Plane {
    gfx:  Object,
    body: Rc<RefCell<RigidBody>>,
}

impl Plane {
    pub fn new(body:   Rc<RefCell<RigidBody>>,
               pos:    &Vec3<f32>,
               normal: &Vec3<f32>,
               color:  Vec3<f32>,
               window: &mut window::Window) -> Plane {

        let mut res = Plane {
            gfx:  window.add_quad(100.0, 100.0, 10, 10),
            body: body
        };

        res.gfx.set_color(color.x, color.y, color.z);
        res.gfx.look_at_z(pos, normal, &Vec3::new(1.0, 0.0, 0.0));

        res.update();

        res
    }

    pub fn select(&mut self) {
    }

    pub fn unselect(&mut self) {
    }

    pub fn update(&mut self) {
        // FIXME: atm we assume the plane does not move
    }

    pub fn object<'r>(&'r self) -> &'r Object {
        &self.gfx
    }
}
