use sfml::system::{Vector2f, Vector2i};
use sfml::graphics::RenderTarget;
use sfml::graphics;
use sfml::window::event;
use na;
use draw_helper::DRAW_SCALE;

static ZOOM_FACTOR: f32 = 0.1;

pub struct Camera {
    pressing:  bool,
    ui:        graphics::View,
    scene:     graphics::View,
    lastx:     i32,
    lasty:     i32,
    curr_zoom: f32
}

impl Camera {
    pub fn new(rw: &graphics::RenderWindow) -> Camera {
        let mut res = Camera {
            pressing:  false,
            ui:        graphics::View::new().unwrap(),
            scene:     graphics::View::new().unwrap(),
            lastx:     0,
            lasty:     0,
            curr_zoom: 1.0
        };

        res.scene.set_center(&Vector2f::new(0.0 as f32, 0.0 as f32));

        let sz = rw.get_size();

        res.set_size(sz.x, sz.y);

        res
    }

    pub fn activate_ui(&self, rw: &mut graphics::RenderWindow) {
        rw.set_view(&self.ui)
    }

    pub fn activate_scene(&self, rw: &mut graphics::RenderWindow) {
        rw.set_view(&self.scene)
    }

    pub fn handle_event(&mut self, event: &event::Event) {
        match *event {
            event::MouseWheelMoved{delta, ..} => {
                let ndelta = delta as f32; // between -1.0 and 1.0

                self.curr_zoom *= 1.0 - ndelta * ZOOM_FACTOR;
                self.scene.zoom(1.0 - ndelta * ZOOM_FACTOR);
            },
            event::MouseButtonPressed{x, y, ..} => {
                self.lastx    = x;
                self.lasty    = y;
                self.pressing = true;
            },
            event::MouseButtonReleased{..} => {
                self.pressing = false;
            },
            event::MouseMoved{x, y} => {
                if self.pressing {
                    let zoom = na::abs(&self.curr_zoom);
                    let zx   = (self.lastx - x) as f32 * zoom;
                    let zy   = (self.lasty - y) as f32 * zoom;

                    self.scene.move_(&Vector2f { x: zx, y: zy });

                    self.lastx = x;
                    self.lasty = y;
                }
            },
            event::Resized{width, height} => self.set_size(width, height),
            _ => {}
        }
    }

    fn set_size(&mut self, width: u32, height: u32) {
        self.scene.set_size(&Vector2f::new(self.curr_zoom * width as f32, self.curr_zoom * height as f32));
        self.ui.set_size(&Vector2f::new(self.curr_zoom * width as f32, self.curr_zoom * height as f32));
    }

    pub fn map_pixel_to_coords(&mut self, pixel_pos: Vector2i) -> Vector2f {
        let center = self.scene.get_center();
        let size   = self.scene.get_size();
        let mapped_coords = Vector2f::new(
            (pixel_pos.x as f32 * self.curr_zoom - (size.x / 2.0) + center.x) / DRAW_SCALE,
            (pixel_pos.y as f32 * self.curr_zoom - (size.y / 2.0) + center.y) / DRAW_SCALE
        );
        mapped_coords
    }
}
