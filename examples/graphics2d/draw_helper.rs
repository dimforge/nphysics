use rsfml::graphics;
use rsfml::graphics::{Vertex, VertexArray, Color};
use rsfml::system::vector2::Vector2f;
use nalgebra::na::Vec2;
use nalgebra::na;
use nphysics::world::World;
use nphysics::detection::constraint::{RBRB, BallInSocket, Fixed};
use nphysics::detection::joint::Joint;

pub static DRAW_SCALE: f32 = 20.0;

pub fn draw_colls(window:  &graphics::RenderWindow,
                  physics: &mut World) {

    let mut collisions = Vec::new();

    physics.interferences(&mut collisions);

    for c in collisions.iter() {
        match *c {
            RBRB(_, _, c) => {
                draw_line(
                    window,
                    &c.world1,
                    &c.world2,
                    &Color::new_RGB(255, 255, 255));

                let center = (c.world1 + c.world2) / 2.0f32;
                draw_line(
                    window,
                    &center,
                    &(center + c.normal * c.depth),
                    &Color::new_RGB(255, 0, 0));

                draw_line(
                    window,
                    &center,
                    &(center + c.normal),
                    &Color::new_RGB(0, 0, 255));
            },
            BallInSocket(ref bis) => {
                draw_line(
                    window,
                    &bis.borrow().anchor1_pos(),
                    &bis.borrow().anchor2_pos(),
                    &Color::new_RGB(255, 0, 0)
                );
            },
            Fixed(ref bis) => {
                draw_line(
                    window,
                    &na::translation(&bis.borrow().anchor1_pos()),
                    &na::translation(&bis.borrow().anchor2_pos()),
                    &Color::new_RGB(255, 0, 0)
                );
            }
        }
    }
}

pub fn draw_line(window: &graphics::RenderWindow, v1: &Vec2<f32>, v2: &Vec2<f32>, color: &Color) {
    let mut vertices = VertexArray::new().unwrap();

    vertices.append(&Vertex::new(
            &Vector2f::new(v1.x.to_f32().unwrap() * DRAW_SCALE, v1.y.to_f32().unwrap() * DRAW_SCALE),
            color,
            &Vector2f::new(0.0, 0.0)));

    vertices.append(&Vertex::new(
            &Vector2f::new(v2.x.to_f32().unwrap() * DRAW_SCALE, v2.y.to_f32().unwrap() * DRAW_SCALE),
            color,
            &Vector2f::new(0.0, 0.0)));

    vertices.set_primitive_type(graphics::Lines);

    window.draw_vertex_array(&vertices);
}
