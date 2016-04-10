use std::sync::Arc;
use sfml::graphics;
use sfml::graphics::Color;
use na::{Pnt2, Pnt3, Iso2};
use nphysics2d::object::{WorldObject, WorldObjectBorrowed};
use draw_helper::draw_line;

pub struct Lines {
    color: Pnt3<u8>,
    base_color: Pnt3<u8>,
    delta: Iso2<f32>,
    object: WorldObject<f32>,
    indices: Arc<Vec<Pnt2<usize>>>,
    vertices: Arc<Vec<Pnt2<f32>>>
}

impl Lines {
    pub fn new(object:   WorldObject<f32>,
               delta:    Iso2<f32>,
               vertices: Arc<Vec<Pnt2<f32>>>,
               indices:  Arc<Vec<Pnt2<usize>>>,
               color:    Pnt3<u8>)
               -> Lines {
        Lines {
            color: color,
            base_color: color,
            delta: delta,
            object: object,
            vertices: vertices,
            indices: indices
        }
    }
}

impl Lines {
    pub fn update(&mut self) {
    }

    pub fn draw(&self, rw: &mut graphics::RenderWindow) {
        let object    = self.object.borrow();
        let transform = object.position() * self.delta;
        let mut color = Color::new_rgb(self.color.x, self.color.y, self.color.z);
        
        if let WorldObjectBorrowed::RigidBody(rb) = object {
            if !rb.is_active() {
                color = Color::new_rgb(self.color.x / 4, self.color.y / 4, self.color.z / 4);
            }
        }

        let vs = &*self.vertices;

        for is in self.indices.iter() {
            let gsv0 = transform * vs[is.x];
            let gsv1 = transform * vs[is.y];
            draw_line(rw, &gsv0, &gsv1, &color);
        }
    }

    pub fn set_color(&mut self, color: Pnt3<u8>) {
        self.color      = color;
        self.base_color = color;
    }

    pub fn select(&mut self) {
        self.color = Pnt3::new(200, 0, 0);
    }

    pub fn unselect(&mut self) {
        self.color = self.base_color;
    }
}
