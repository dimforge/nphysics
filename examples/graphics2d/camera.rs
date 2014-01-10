use extra::arc::RWArc;
use rsfml::system::vector2::Vector2f;
use rsfml::graphics::view;
use rsfml::graphics::render_window;
use rsfml::window::event;

static ZOOM_FACTOR: f32 = 0.1;

pub struct Camera {
    priv pressing:  bool,
    priv ui:        RWArc<view::View>,
    priv scene:     RWArc<view::View>,
    priv lastx:     int,
    priv lasty:     int,
    priv curr_zoom: f32
}

impl Camera {
    pub fn new() -> Camera {
        let mut scene = view::View::new().unwrap();

        scene.set_center(&Vector2f::new(0.0 as f32, 0.0 as f32));

        Camera {
            pressing:  false,
            ui:        RWArc::new(view::View::new().unwrap()),
            scene:     RWArc::new(scene),
            lastx:     0,
            lasty:     0,
            curr_zoom: 1.0
        }
    }

    pub fn activate_ui(&self, rw: &mut render_window::RenderWindow) {
        rw.set_view(&self.ui)
    }

    pub fn activate_scene(&self, rw: &mut render_window::RenderWindow) {
        rw.set_view(&self.scene)
    }

    pub fn handle_event(&mut self, event: &event::Event) {
        match *event {
            event::MouseWheelMoved{delta, ..} => {
                let ndelta = delta as f32; // between -1.0 and 1.0

                self.curr_zoom *= (1.0 + ndelta * ZOOM_FACTOR);
                self.scene.write(|s| s.zoom(1.0 + ndelta * ZOOM_FACTOR));
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

                    self.scene.write(|s| s.move(&Vector2f { x: zx, y: zy }));

                    self.lastx = x;
                    self.lasty = y;
                }
            }
            event::Resized{width, height} => {
                self.scene.write(|s| s.set_size(&Vector2f::new(self.curr_zoom * width as f32, self.curr_zoom * height as f32)));
                self.ui.write(|s| s.set_size(&Vector2f::new(self.curr_zoom * width as f32, self.curr_zoom * height as f32)));
            }
            _ => {}
        }
    }
}
