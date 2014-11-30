use std::rc::Rc;
use std::cell::RefCell;
use rsfml::graphics;
use rsfml::graphics::Color;
use na::{Pnt2, Pnt3, Iso2};
use nphysics::object::RigidBody;
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

        let ga = transform * self.a;
        let gb = transform * self.b;
        draw_line(rw, &ga, &gb, &Color::new_RGB(self.color.x, self.color.y, self.color.z));
    }

    pub fn select(&mut self) {
        self.color = Pnt3::new(200, 0, 0);
    }

    pub fn unselect(&mut self) {
        self.color = self.base_color;
    }
}
