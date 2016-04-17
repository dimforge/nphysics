use sfml::graphics;
use sfml::graphics::{RectangleShape, Color, RenderTarget, Shape, Transformable};
use sfml::system::Vector2f;
use na::{Point3, Isometry2};
use nphysics2d::object::WorldObject;
use objects;
use draw_helper::DRAW_SCALE;

pub struct Box<'a> {
    color: Point3<u8>,
    base_color: Point3<u8>,
    delta: Isometry2<f32>,
    object: WorldObject<f32>,
    gfx: RectangleShape<'a>
}

impl<'a> Box<'a> {
    pub fn new(object: WorldObject<f32>,
               delta:  Isometry2<f32>,
               rx:     f32,
               ry:     f32,
               color:  Point3<u8>) -> Box<'a> {
        let mut res = Box {
            color: color,
            base_color: color,
            delta: delta,
            gfx: RectangleShape::new().unwrap(),
            object: object
        };

        let drx = rx as f32 * DRAW_SCALE;
        let dry = ry as f32 * DRAW_SCALE;

        res.gfx.set_fill_color(&Color::new_rgb(color.x, color.y, color.z));
        res.gfx.set_size(&Vector2f { x: drx * 2.0, y: dry * 2.0 });
        res.gfx.set_origin(&Vector2f { x: drx, y: dry });

        res
    }
}

impl<'a> Box<'a> {
    pub fn update(&mut self) {
        objects::update_scene_node(&mut self.gfx, &self.object, &self.color, &self.delta)
    }

    pub fn draw(&self, rw: &mut graphics::RenderWindow) {
        rw.draw(&self.gfx);
    }

    pub fn set_color(&mut self, color: Point3<u8>) {
        self.color      = color;
        self.base_color = color;
    }

    pub fn select(&mut self) {
        self.color = Point3::new(200, 0, 0);
    }

    pub fn unselect(&mut self) {
        self.color = self.base_color;
    }
}
