use std::rc::Rc;
use std::cell::RefCell;
use kiss3d::window::Window;
use kiss3d::object::Object;
use kiss3d::mesh;
use nalgebra::na::{Vec3, Iso3, Transformation};
use nalgebra::na;
use nphysics::object::RigidBody;

pub struct Mesh {
    priv color:      Vec3<f32>,
    priv base_color: Vec3<f32>,
    priv delta:      Iso3<f32>,
    priv gfx:        Object,
    priv body:       Rc<RefCell<RigidBody>>
}

impl Mesh {
    pub fn new(body:     Rc<RefCell<RigidBody>>,
               delta:    Iso3<f32>,
               vertices: ~[Vec3<f32>],
               indices:  ~[Vec3<u32>],
               color:    Vec3<f32>,
               window:   &mut Window) -> Mesh {
        let vs = vertices;
        let is = indices;

        let mesh = mesh::Mesh::new(vs, is, None, None, false);

        let mut res = Mesh {
            color:      color,
            base_color: color,
            delta:      delta,
            gfx:        window.add_mesh(mesh, 1.0),
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
        let rb = self.body.borrow().borrow();

        if rb.get().is_active() {
            {
                self.gfx.set_transformation(na::transformation(rb.get()) * self.delta);
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
