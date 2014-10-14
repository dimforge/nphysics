use std::rc::Rc;
use std::cell::RefCell;
use sync::Arc;
use rsfml::graphics;
use rsfml::graphics::Color;
use na::{Pnt2, Pnt3, Iso2};
use nphysics::object::RigidBody;
use draw_helper::draw_line;

pub struct Lines {
    color: Pnt3<u8>,
    base_color: Pnt3<u8>,
    delta: Iso2<f32>,
    body: Rc<RefCell<RigidBody>>,
    indices: Arc<Vec<uint>>,
    vertices: Arc<Vec<Pnt2<f32>>>
}

impl Lines {
    pub fn new(body:     Rc<RefCell<RigidBody>>,
               delta:    Iso2<f32>,
               vertices: Arc<Vec<Pnt2<f32>>>,
               indices:  Arc<Vec<uint>>,
               color:    Pnt3<u8>) -> Lines {
        Lines {
            color: color,
            base_color: color,
            delta: delta,
            body: body,
            vertices: vertices,
            indices: indices
        }
    }
}

impl Lines {
    pub fn update(&mut self) {
    }

    pub fn draw(&self, rw: &mut graphics::RenderWindow) {
        let body      = self.body.borrow();
        let transform = body.transform_ref() * self.delta;

        let vs = self.vertices.deref();

        for is in self.indices.as_slice().chunks(2) {
            let gsv0 = transform * vs[is[0]];
            let gsv1 = transform * vs[is[1]];
            draw_line(rw, &gsv0, &gsv1, &Color::new_RGB(self.color.x, self.color.y, self.color.z));
        }
    }

    pub fn select(&mut self) {
        self.color = Pnt3::new(200, 0, 0);
    }

    pub fn unselect(&mut self) {
        self.color = self.base_color;
    }
}
