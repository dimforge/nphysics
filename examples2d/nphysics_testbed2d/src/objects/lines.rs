use std::rc::Rc;
use std::cell::RefCell;
use std::sync::Arc;
use sfml::graphics;
use sfml::graphics::Color;
use na::{Pnt2, Pnt3, Iso2};
use nphysics2d::object::RigidBody;
use draw_helper::draw_line;

pub struct Lines {
    color: Pnt3<u8>,
    base_color: Pnt3<u8>,
    delta: Iso2<f32>,
    body: Rc<RefCell<RigidBody<f32>>>,
    indices: Arc<Vec<Pnt2<usize>>>,
    vertices: Arc<Vec<Pnt2<f32>>>
}

impl Lines {
    pub fn new(body:     Rc<RefCell<RigidBody<f32>>>,
               delta:    Iso2<f32>,
               vertices: Arc<Vec<Pnt2<f32>>>,
               indices:  Arc<Vec<Pnt2<usize>>>,
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
        let transform = *body.position() * self.delta;
        let color = match body.is_active() {
            true  => Color::new_rgb(self.color.x, self.color.y, self.color.z),
            false => Color::new_rgb(self.color.x / 4, self.color.y / 4, self.color.z / 4)
        };

        let vs = &*self.vertices;

        for is in self.indices.iter() {
            let gsv0 = transform * vs[is.x];
            let gsv1 = transform * vs[is.y];
            draw_line(rw, &gsv0, &gsv1, &color);
        }
    }

    pub fn select(&mut self) {
        self.color = Pnt3::new(200, 0, 0);
    }

    pub fn unselect(&mut self) {
        self.color = self.base_color;
    }
}
