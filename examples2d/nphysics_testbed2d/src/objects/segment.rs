use std::rc::Rc;
use std::cell::RefCell;
use sfml::graphics;
use sfml::graphics::Color;
use na::{Pnt2, Pnt3, Iso2};
use nphysics2d::object::RigidBody;
use draw_helper::draw_line;

pub struct Segment {
    color: Pnt3<u8>,
    base_color: Pnt3<u8>,
    delta: Iso2<f32>,
    body:  Rc<RefCell<RigidBody>>,
    a:     Pnt2<f32>,
    b:     Pnt2<f32>,
}

impl Segment {
    pub fn new(body:     Rc<RefCell<RigidBody>>,
               delta:    Iso2<f32>,
               a:        Pnt2<f32>,
               b:        Pnt2<f32>,
               color:    Pnt3<u8>) -> Segment {
        Segment {
            color: color,
            base_color: color,
            delta: delta,
            body: body,
            a:    a,
            b:    b
        }
    }
}

impl Segment {
    pub fn update(&mut self) {
    }

    pub fn draw(&self, rw: &mut graphics::RenderWindow) {
        let body      = self.body.borrow();
        let transform = *body.position() * self.delta;
        let color = match body.is_active() {
            true  => Color::new_rgb(self.color.x, self.color.y, self.color.z),
            false => Color::new_rgb(self.color.x / 4, self.color.y / 4, self.color.z / 4)
        };

        let ga = transform * self.a;
        let gb = transform * self.b;
        draw_line(rw, &ga, &gb, &color);
    }

    pub fn select(&mut self) {
        self.color = Pnt3::new(200, 0, 0);
    }

    pub fn unselect(&mut self) {
        self.color = self.base_color;
    }
}
