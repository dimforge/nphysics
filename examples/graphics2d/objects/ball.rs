use rsfml::graphics::render_window;
use rsfml::graphics::circle_shape::CircleShape;
use rsfml::graphics::color;
use rsfml::system::vector2;
use nalgebra::traits::translation::Translation;
use nalgebra::traits::rotation::Rotation;
use nalgebra::vec::Vec3;
use engine::SceneNode;
use draw_helper::DRAW_SCALE;
use nphysics::aliases::dim2;

struct Ball {
    priv color: Vec3<u8>,
    priv delta: dim2::Transform2d<f64>,
    priv body:  @mut dim2::Body2d<f64>,
    priv gfx:   CircleShape
}

impl Ball {
    pub fn new(body:   @mut dim2::Body2d<f64>,
               delta:  dim2::Transform2d<f64>,
               radius: f64,
               color:  Vec3<u8>) -> Ball {
        let dradius = radius as f32 * DRAW_SCALE;

        let mut res = Ball {
            color: color,
            delta: delta,
            gfx:   CircleShape::new().unwrap(),
            body:  body
        };

        res.gfx.set_fill_color(&color::Color::new_from_RGB(color.x, color.y, color.z));
        res.gfx.set_radius(dradius as float);
        res.gfx.set_origin(&vector2::Vector2f { x: dradius, y: dradius }); 

        res
    }
}

impl SceneNode for Ball {
    fn update(&mut self) {
        let body = self.body.to_rigid_body_or_fail();
        let transform = body.transform_ref() * self.delta;
        let pos = transform.translation();
        let rot = transform.rotation();

        self.gfx.set_position(&vector2::Vector2f {
            x: pos.x as f32 * DRAW_SCALE,
            y: pos.y as f32 * DRAW_SCALE
        });
        self.gfx.set_rotation(rot.x.to_degrees() as float);

        if body.is_active() {
            self.gfx.set_fill_color(
                &color::Color::new_from_RGB(self.color.x, self.color.y, self.color.z));
        }
        else {
            self.gfx.set_fill_color(
                &color::Color::new_from_RGB(self.color.x / 4, self.color.y / 4, self.color.z / 4));
        }
    }

    fn draw(&self, rw: &mut render_window::RenderWindow) {
        rw.draw(&self.gfx);
    }
}
