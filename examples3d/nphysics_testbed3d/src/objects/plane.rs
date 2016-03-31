use std::rc::Rc;
use std::cell::RefCell;
use na;
use kiss3d::window;
use kiss3d::scene::SceneNode;
use na::{Pnt3, Vec3};
use nphysics3d::object::RigidBody;

pub struct Plane {
    gfx:  SceneNode,
    body: Rc<RefCell<RigidBody>>
}

impl Plane {
    pub fn new(body:         Rc<RefCell<RigidBody>>,
               world_pos:    &Pnt3<f32>,
               world_normal: &Vec3<f32>,
               color:        Pnt3<f32>,
               window:       &mut window::Window) -> Plane {
        let mut res = Plane {
            gfx:  window.add_quad(100.0, 100.0, 10, 10),
            body: body
        };

        res.gfx.set_color(color.x, color.y, color.z);

        let up;

        if na::is_zero(&world_normal.z) && na::is_zero(&world_normal.y) {
            up = Vec3::z();
        }
        else {
            up = Vec3::x();
        }

        res.gfx.reorient(world_pos, &(*world_pos + *world_normal), &up);

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

    pub fn object(&self) -> &SceneNode {
        &self.gfx
    }

    pub fn body(&self) -> &Rc<RefCell<RigidBody>> {
        &self.body
    }
}
