use rsfml::system::vector2::{Vector2f, Vector2i};
use rsfml::graphics::RenderTarget;
use rsfml::graphics;
use rsfml::window::event;
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
    pub fn new() -> Camera {
        let mut scene = graphics::View::new().unwrap();

        scene.set_center(&Vector2f::new(0.0 as f32, 0.0 as f32));

        Camera {
            pressing:  false,
            ui:        graphics::View::new().unwrap(),
            scene:     scene,
            lastx:     0,
            lasty:     0,
            curr_zoom: 1.0
        }
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

                self.curr_zoom *= 1.0 + ndelta * ZOOM_FACTOR;
                self.scene.zoom(1.0 + ndelta * ZOOM_FACTOR);
            }
            event::MouseButtonPressed{x, y, ..}  => {
                self.lastx    = x;
                self.lasty    = y;
                self.pressing = true;
            }
            event::MouseButtonReleased{..}  => {
                self.pressing = false;
            }
            event::MouseMoved{x, y}             => {
                if self.pressing {
                    let zoom = self.curr_zoom.abs();
                    let zx   = (self.lastx - x) as f32 * zoom;
                    let zy   = (self.lasty - y) as f32 * zoom;

                    self.scene.move_(&Vector2f { x: zx, y: zy });

                    self.lastx = x;
                    self.lasty = y;
                }
            }
            event::Resized{width, height} => {
                self.scene.set_size(&Vector2f::new(self.curr_zoom * width as f32, self.curr_zoom * height as f32));
                self.ui.set_size(&Vector2f::new(self.curr_zoom * width as f32, self.curr_zoom * height as f32));
            }
            _ => {}
        }
    }

    pub fn map_pixel_to_coords(&mut self, pixel_pos: Vector2i) -> Vector2f {
        let center = self.scene.get_center();
        let size = self.scene.get_size();
        let mapped_coords = Vector2f::new(
            (pixel_pos.x as f32 * self.curr_zoom - (size.x / 2.0) + center.x) / DRAW_SCALE,
            (pixel_pos.y as f32 * self.curr_zoom - (size.y / 2.0) + center.y) / DRAW_SCALE
        );
        mapped_coords
    }
}
