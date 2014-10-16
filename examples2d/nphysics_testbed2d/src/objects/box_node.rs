use std::rc::Rc;
use std::cell::RefCell;
use rsfml::graphics;
use rsfml::graphics::{RectangleShape, Color, RenderTarget};
use rsfml::system::vector2;
use na::{Pnt3, Iso2};
use na;
use nphysics::object::RigidBody;
use draw_helper::DRAW_SCALE;

pub struct Box<'a> {
    color: Pnt3<u8>,
    base_color: Pnt3<u8>,
    delta: Iso2<f32>,
    body: Rc<RefCell<RigidBody>>,
    gfx: RectangleShape<'a>
}

impl<'a> Box<'a> {
    pub fn new(body:  Rc<RefCell<RigidBody>>,
               delta: Iso2<f32>,
               rx:    f32,
               ry:    f32,
               color: Pnt3<u8>) -> Box<'a> {
        let mut res = Box {
            color: color,
            base_color: color,
            delta: delta,
            gfx: RectangleShape::new().unwrap(),
            body: body
        };

        let drx = rx as f32 * DRAW_SCALE;
        let dry = ry as f32 * DRAW_SCALE;

        res.gfx.set_fill_color(&Color::new_RGB(color.x, color.y, color.z));
        res.gfx.set_size(&vector2::Vector2f { x: drx * 2.0, y: dry * 2.0 });
        res.gfx.set_origin(&vector2::Vector2f { x: drx, y: dry });

        res
    }
}

impl<'a> Box<'a> {
    pub fn update(&mut self) {
        let body     = self.body.borrow();
        let transform = body.transform_ref() * self.delta;
        let pos = na::translation(&transform);
        let rot = na::rotation(&transform);

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

    pub fn draw(&self, rw: &mut graphics::RenderWindow) {
        rw.draw(&self.gfx);
    }

    pub fn select(&mut self) {
        self.color = Pnt3::new(200, 0, 0);
    }

    pub fn unselect(&mut self) {
        self.color = self.base_color;
    }
}
