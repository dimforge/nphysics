use std::rc::Rc;
use std::cell::RefCell;
use kiss3d::window;
use kiss3d::scene::SceneNode;
use na::{Pnt3, Iso3};
use nphysics3d::object::RigidBody;

pub struct Box {
    color:      Pnt3<f32>,
    base_color: Pnt3<f32>,
    delta:      Iso3<f32>,
    gfx:        SceneNode,
    body:       Rc<RefCell<RigidBody<f32>>>,
}

impl Box {
    pub fn new(body:   Rc<RefCell<RigidBody<f32>>>,
               delta:  Iso3<f32>,
               rx:     f32,
               ry:     f32,
               rz:     f32,
               color:  Pnt3<f32>,
                       window: &mut window::Window) -> Box {
        let gx = rx * 2.0;
        let gy = ry * 2.0;
        let gz = rz * 2.0;
        let t  = body.borrow().position().clone();

        let mut res = Box {
            color:      color,
            base_color: color,
            delta:      delta,
            gfx:        window.add_cube(gx, gy, gz),
            body:       body
        };

        res.gfx.set_color(color.x, color.y, color.z);
        res.gfx.set_local_transformation(t * res.delta);
        res.update();

        res
    }

    pub fn select(&mut self) {
        self.color = Pnt3::new(1.0, 0.0, 0.0);
    }

    pub fn unselect(&mut self) {
        self.color = self.base_color;
    }

    pub fn update(&mut self) {
        let rb = self.body.borrow();

        if rb.is_active() {
            self.gfx.set_local_transformation(*rb.position() * self.delta);
            self.gfx.set_color(self.color.x, self.color.y, self.color.z);
        }
        else {
            self.gfx.set_color(self.color.x * 0.25, self.color.y * 0.25, self.color.z * 0.25);
        }
    }

    pub fn object(&self) -> &SceneNode {
        &self.gfx
    }

    pub fn body(&self) -> &Rc<RefCell<RigidBody<f32>>> {
        &self.body
    }
}
