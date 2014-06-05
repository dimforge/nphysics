use std::rc::Rc;
use std::cell::RefCell;
use kiss3d::window::Window;
use kiss3d::scene::SceneNode;
use nalgebra::na::{Vec3, Iso3};
use nalgebra::na;
use nphysics::object::RigidBody;
use ncollide::procedural;

pub struct BezierSurface {
    color:      Vec3<f32>,
    base_color: Vec3<f32>,
    delta:      Iso3<f32>,
    gfx:        SceneNode,
    body:       Rc<RefCell<RigidBody>>
}

impl BezierSurface {
    pub fn new(body:           Rc<RefCell<RigidBody>>,
               delta:          Iso3<f32>,
               control_points: &[Vec3<f32>],
               nupoints:       uint,
               nvpoints:       uint,
               color:          Vec3<f32>,
               window:         &mut Window) -> BezierSurface {
        let t      = na::transformation(body.borrow().deref());
        let bezier = procedural::bezier_surface(control_points, nupoints, nvpoints, 100, 100);

        let mut res = BezierSurface {
            color:      color,
            base_color: color,
            delta:      delta,
            gfx:        window.add_trimesh(bezier, na::one()),
            body:       body
        };

        res.gfx.set_color(color.x, color.y, color.z);
        res.gfx.set_local_transformation(t * res.delta);
        res.gfx.enable_backface_culling(false);
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
        &'r self.gfx
    }

    pub fn object_mut<'r>(&'r mut self) -> &'r mut SceneNode {
        &'r mut self.gfx
    }
}
