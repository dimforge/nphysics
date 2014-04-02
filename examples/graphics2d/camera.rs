use std::cell::RefCell;
use std::rc::Rc;
use rsfml::system::vector2::Vector2f;
use rsfml::graphics;
use rsfml::window::event;

static ZOOM_FACTOR: f32 = 0.1;

pub struct Camera {
    pressing:  bool,
    ui:        Rc<RefCell<graphics::View>>,
    scene:     Rc<RefCell<graphics::View>>,
    lastx:     int,
    lasty:     int,
    curr_zoom: f32
}

impl Camera {
    pub fn new() -> Camera {
        let mut scene = graphics::View::new().unwrap();

        scene.set_center(&Vector2f::new(0.0 as f32, 0.0 as f32));

        Camera {
            pressing:  false,
            ui:        Rc::new(RefCell::new(graphics::View::new().unwrap())),
            scene:     Rc::new(RefCell::new(scene)),
            lastx:     0,
            lasty:     0,
            curr_zoom: 1.0
        }
    }

    pub fn activate_ui(&self, rw: &mut graphics::RenderWindow) {
        rw.set_view(self.ui.clone())
    }

    pub fn activate_scene(&self, rw: &mut graphics::RenderWindow) {
        rw.set_view(self.scene.clone())
    }

    pub fn handle_event(&mut self, event: &event::Event) {
        match *event {
            event::MouseWheelMoved{delta, ..} => {
                let ndelta = delta as f32; // between -1.0 and 1.0

                self.curr_zoom *= 1.0 + ndelta * ZOOM_FACTOR;
                self.scene.borrow_mut().zoom(1.0 + ndelta * ZOOM_FACTOR);
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

                    self.scene.borrow_mut().move(&Vector2f { x: zx, y: zy });

                    self.lastx = x;
                    self.lasty = y;
                }
            }
            event::Resized{width, height} => {
                self.scene.borrow_mut().set_size(&Vector2f::new(self.curr_zoom * width as f32, self.curr_zoom * height as f32));
                self.ui.borrow_mut().set_size(&Vector2f::new(self.curr_zoom * width as f32, self.curr_zoom * height as f32));
            }
            _ => {}
        }
    }
}
