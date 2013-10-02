use rsfml::system::vector2;
use rsfml::traits::drawable::Drawable;
use rsfml::graphics::font::Font;
use rsfml::graphics::render_window;
use rsfml::graphics::text::Text;
use rsfml::graphics::Color;
use extra::time;

pub struct Fps<'self> {
    priv delta:     f64,
    priv last_time: f64,
    priv fps:       Text<'self>
}

impl<'self> Fps<'self> {
    pub fn new(font: &'self Font) -> Fps<'self> {
        let mut txt = Text::new().unwrap();
        txt.set_font(font);
        txt.set_position(&vector2::Vector2f { x: 0.0, y: 0.0 });
        txt.set_color(&Color::new_RGB(255, 255, 255));

        Fps {
            delta:     0.0,
            last_time: 0.0,
            fps:       txt
        }
    }

    pub fn reset(&mut self) {
        self.last_time = time::precise_time_s();
    }

    pub fn register_delta(&mut self) {
        self.delta = self.elapsed_seconds()
    }

    pub fn registred_delta(&self) -> f64 {
        self.delta
    }

    pub fn elapsed_seconds(&self) -> f64 {
        time::precise_time_s() - self.last_time
    }

    pub fn draw_elapsed(&mut self, rw: &render_window::RenderWindow) {
        let elapsed = self.elapsed_seconds();

        self.fps.set_string(elapsed.to_str());
        self.fps.draw_in_render_window(rw);
    }

    pub fn draw_registered(&mut self, rw: &render_window::RenderWindow) {
        let elapsed = self.delta;

        self.fps.set_position(
            &rw.map_pixel_to_coords(
                &vector2::Vector2i { x: 0, y : 0 },
                rw.get_view()
            )
        );
        self.fps.set_string(elapsed.to_str());
        self.fps.draw_in_render_window(rw);
    }
}
