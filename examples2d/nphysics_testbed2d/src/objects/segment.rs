use sfml::graphics;
use sfml::graphics::Color;
use na::{Pnt2, Pnt3, Iso2};
use nphysics2d::object::{WorldObject, WorldObjectBorrowed};
use draw_helper::draw_line;

pub struct Segment {
    color: Pnt3<u8>,
    base_color: Pnt3<u8>,
    delta: Iso2<f32>,
    object:  WorldObject<f32>,
    a:     Pnt2<f32>,
    b:     Pnt2<f32>,
}

impl Segment {
    pub fn new(object: WorldObject<f32>,
               delta:  Iso2<f32>,
               a:      Pnt2<f32>,
               b:      Pnt2<f32>,
               color:  Pnt3<u8>)
               -> Segment {
        Segment {
            color: color,
            base_color: color,
            delta: delta,
            object: object,
            a:    a,
            b:    b
        }
    }
}

impl Segment {
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

        let ga = transform * self.a;
        let gb = transform * self.b;
        draw_line(rw, &ga, &gb, &color);
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
