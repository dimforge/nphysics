use std::num::NumCast;
use rsfml::graphics::render_window;
use rsfml::graphics::vertex::Vertex;
use rsfml::graphics::vertex_array::VertexArray;
use rsfml::graphics::color;
use rsfml::graphics::primitive_type;
use rsfml::system::vector2;
use nphysics::aliases::dim2;
use nphysics::detection::constraint::RBRB;

pub static DRAW_SCALE: f32 = 20.0;

pub fn draw_colls(window:  &render_window::RenderWindow,
                  physics: &mut dim2::World2d<f64>) {

    let mut collisions = ~[];

    for c in physics.detectors().iter() {
        c.interferences(&mut collisions);
    }

    for c in collisions.iter() {
        match *c {
            RBRB(_, _, c) => {
                let mut vertices = VertexArray::new().unwrap();
                let w1           = c.world1;
                let w2           = c.world2;

                vertices.append(&Vertex::new(
                        &vector2::Vector2f {
                            x: w1.x.to_f32() * DRAW_SCALE,
                            y: w1.y.to_f32() * DRAW_SCALE
                        },
                        &color::Color::new_from_RGB(255, 255, 255),
                        &vector2::Vector2f { x: 0.0, y: 0.0 }
                        ));

                vertices.append(&Vertex::new(
                        &vector2::Vector2f {
                            x: w2.x.to_f32() * DRAW_SCALE,
                            y: w2.y.to_f32() * DRAW_SCALE
                        },
                        &color::Color::new_from_RGB(255, 255, 255),
                        &vector2::Vector2f { x: 0.0, y: 0.0 }
                        ));

                vertices.set_primitive_type(primitive_type::Lines);

                window.draw_vertex_array(&vertices);
            }
        }
    }
}
