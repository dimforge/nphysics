use rsfml::graphics::render_window;
use rsfml::graphics::rectangle_shape::RectangleShape;
use rsfml::graphics::Color;
use rsfml::system::vector2;
use nalgebra::mat::{Translation, Rotation};
use nalgebra::vec::Vec3;
use nphysics::aliases::dim2;
use draw_helper::DRAW_SCALE;

struct Box<'self> {
    priv color: Vec3<u8>,
    priv delta: dim2::Transform2d<f64>,
    priv body:  @mut dim2::Body2d<f64>,
    priv gfx:   RectangleShape<'self>
}

impl<'self> Box<'self> {
    pub fn new(body:  @mut dim2::Body2d<f64>,
               delta: dim2::Transform2d<f64>,
               rx:    f64,
               ry:    f64,
               color: Vec3<u8>) -> Box {
        let mut res = Box {
            color: color,
            delta: delta,
            gfx:   RectangleShape::new().unwrap(),
            body:  body
        };

        let drx = rx as f32 * DRAW_SCALE;
        let dry = ry as f32 * DRAW_SCALE;

        res.gfx.set_fill_color(&Color::new_RGB(color.x, color.y, color.z));
        res.gfx.set_size(&vector2::Vector2f { x: drx * 2.0, y: dry * 2.0 });
        res.gfx.set_origin(&vector2::Vector2f { x: drx, y: dry });

        res
    }
}

impl<'self> Box<'self> {
    pub fn update(&mut self) {
        let body = self.body.to_rigid_body_or_fail();
        let transform = body.transform_ref() * self.delta;
        let pos = transform.translation();
        let rot = transform.rotation();

        self.gfx.set_position(&vector2::Vector2f {
            x: pos.x as f32 * DRAW_SCALE,
            y: pos.y as f32 * DRAW_SCALE
        });
        self.gfx.set_rotation(rot.x.to_degrees() as f32);

        if body.is_active() {
            self.gfx.set_fill_color(
                &Color::new_RGB(self.color.x, self.color.y, self.color.z));
        }
        else {
            self.gfx.set_fill_color(
                &Color::new_RGB(self.color.x / 4, self.color.y / 4, self.color.z / 4));
        }
    }

    pub fn draw(&self, rw: &mut render_window::RenderWindow) {
        rw.draw(&self.gfx);
    }
}
