use std::rc::Rc;
use std::cell::RefCell;
use std::num::Zero;
use kiss3d::window;
use kiss3d::scene::SceneNode;
use nalgebra::na::Vec3;
use nphysics::object::RigidBody;

pub struct Plane {
    gfx:  SceneNode,
    body: Rc<RefCell<RigidBody>>
}

impl Plane {
    pub fn new(body:         Rc<RefCell<RigidBody>>,
               world_pos:    &Vec3<f32>,
               world_normal: &Vec3<f32>,
               color:        Vec3<f32>,
               window:       &mut window::Window) -> Plane {
        let mut res = Plane {
            gfx:  window.add_quad(100.0, 100.0, 10, 10),
            body: body
        };

        res.gfx.set_color(color.x, color.y, color.z);

        let up;

        if world_normal.z.is_zero() && world_normal.y.is_zero() {
            up = Vec3::z();
        }
        else {
            up = Vec3::x();
        }

        res.gfx.look_at_z(world_pos, &(*world_pos + *world_normal), &up);

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

    pub fn object<'r>(&'r self) -> &'r SceneNode {
        &self.gfx
    }

    pub fn body<'a>(&'a self) -> &'a Rc<RefCell<RigidBody>> {
        &self.body
    }
}
