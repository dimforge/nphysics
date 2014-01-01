use extra::arc::Arc;
use rsfml::graphics::render_window;
use rsfml::graphics::Color;
use nalgebra::na::{Vec2, Vec3, Iso2};
use nphysics::object::Body;
use draw_helper::draw_line;

pub struct Lines {
    priv color:    Vec3<u8>,
    priv delta:    Iso2<f32>,
    priv body:     @mut Body,
    priv indices:  Arc<~[uint]>,
    priv vertices: Arc<~[Vec2<f32>]>
}

impl Lines {
    pub fn new(body:     @mut Body,
               delta:    Iso2<f32>,
               vertices: Arc<~[Vec2<f32>]>,
               indices:  Arc<~[uint]>,
               color:    Vec3<u8>) -> Lines {
        Lines {
            color:    color,
            delta:    delta,
            body:     body,
            vertices: vertices,
            indices:  indices
        }
    }
}

impl Lines {
    pub fn update(&mut self) {
    }

    pub fn draw(&self, rw: &mut render_window::RenderWindow) {
        let body      = self.body.to_rigid_body_or_fail();
        let transform = body.transform_ref() * self.delta;

        let vs = self.vertices.get();

        for is in self.indices.get().chunks(2) {
            let gsv0 = transform * vs[is[0]];
            let gsv1 = transform * vs[is[1]];
            draw_line(rw, &gsv0, &gsv1, &Color::new_RGB(self.color.x, self.color.y, self.color.z));
        }
    }
}
