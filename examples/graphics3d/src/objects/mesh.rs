use std::rc::Rc;
use std::cell::RefCell;
use kiss3d::window::Window;
use kiss3d::scene::SceneNode;
use kiss3d::resource;
use nalgebra::na::{Vec3, Iso3};
use nalgebra::na;
use nphysics::object::RigidBody;

pub struct Mesh {
    color:      Vec3<f32>,
    base_color: Vec3<f32>,
    delta:      Iso3<f32>,
    gfx:        SceneNode,
    body:       Rc<RefCell<RigidBody>>
}

impl Mesh {
    pub fn new(body:     Rc<RefCell<RigidBody>>,
               delta:    Iso3<f32>,
               vertices: Vec<Vec3<f32>>,
               indices:  Vec<Vec3<u32>>,
               color:    Vec3<f32>,
               window:   &mut Window) -> Mesh {
        let vs = vertices;
        let is = indices;

        let mesh = resource::Mesh::new(vs, is, None, None, false);
        let t    = na::transformation(body.borrow().deref());

        let mut res = Mesh {
            color:      color,
            base_color: color,
            delta:      delta,
            gfx:        window.add_mesh(Rc::new(RefCell::new(mesh)), na::one()),
            body:       body
        };

        res.gfx.enable_backface_culling(false);
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
            {
                self.gfx.set_local_transformation(na::transformation(rb.deref()) * self.delta);
            }

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
        &mut self.gfx
    }

    pub fn body<'a>(&'a self) -> &'a Rc<RefCell<RigidBody>> {
        &self.body
    }
}
