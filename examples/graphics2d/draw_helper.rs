use std::num::NumCast;
use rsfml::graphics::render_window;
use rsfml::graphics::vertex::Vertex;
use rsfml::graphics::vertex_array::VertexArray;
use rsfml::graphics::color::Color;
use rsfml::graphics::primitive_type;
use rsfml::system::vector2::Vector2f;
use nalgebra::vec::Vec2;
use nphysics::aliases::dim2;
use nphysics::detection::constraint::{RBRB, BallInSocket};

pub static DRAW_SCALE: f32 = 20.0;

pub fn draw_colls(window:  &render_window::RenderWindow,
                  physics: &mut dim2::BodyWorld2d<f64>) {

    let mut collisions = ~[];

    for c in physics.world().detectors().iter() {
        c.interferences(&mut collisions);
    }

    for c in collisions.iter() {
        match *c {
            RBRB(_, _, c) => {
                draw_line(
                    window,
                    &c.world1,
                    &c.world2,
                    &Color::new_from_RGB(255, 255, 255));
            },
            BallInSocket(bis) => {
                draw_line(
                    window,
                    &bis.anchor1_pos(),
                    &bis.anchor2_pos(),
                    &Color::new_from_RGB(255, 0, 0))
            }
        }
    }
}

pub fn draw_line(window: &render_window::RenderWindow, v1: &Vec2<f64>, v2: &Vec2<f64>, color: &Color) {
    let mut vertices = VertexArray::new().unwrap();

    vertices.append(&Vertex::new(
            &Vector2f::new(v1.x.to_f32() * DRAW_SCALE, v1.y.to_f32() * DRAW_SCALE),
            color,
            &Vector2f::new(0.0, 0.0)));

    vertices.append(&Vertex::new(
            &Vector2f::new(v2.x.to_f32() * DRAW_SCALE, v2.y.to_f32() * DRAW_SCALE),
            color,
            &Vector2f::new(0.0, 0.0)));

    vertices.set_primitive_type(primitive_type::Lines);

    window.draw_vertex_array(&vertices);
}
