use sfml::graphics;
use sfml::graphics::{Vertex, VertexArray, Color, RenderTarget};
use sfml::system::Vector2f;
use na::Translate;
use na::Pnt2;
use na;
use nphysics2d::world::World;
use nphysics2d::detection::constraint::Constraint;
use nphysics2d::detection::joint::Joint;

pub static DRAW_SCALE: f32 = 20.0;

pub fn draw_colls(window:  &mut graphics::RenderWindow,
                  physics: &mut World<f32>) {

    let mut collisions = Vec::new();

    physics.constraints(&mut collisions);

    for c in collisions.iter() {
        match *c {
            Constraint::RBRB(_, _, ref c) => {
                draw_line(
                    window,
                    &c.world1,
                    &c.world2,
                    &Color::new_rgb(255, 255, 255));

                let center = na::center(&c.world1, &c.world2);
                draw_line(
                    window,
                    &center,
                    &(center + c.normal * c.depth),
                    &Color::new_rgb(255, 0, 0));

                draw_line(
                    window,
                    &center,
                    &(center + c.normal),
                    &Color::new_rgb(0, 0, 255));
            },
            Constraint::BallInSocket(ref bis) => {
                draw_line(
                    window,
                    &bis.borrow().anchor1_pos(),
                    &bis.borrow().anchor2_pos(),
                    &Color::new_rgb(255, 0, 0)
                );
            },
            Constraint::Fixed(ref bis) => {
                draw_line(
                    window,
                    &na::translation(&bis.borrow().anchor1_pos()).translate(&na::orig()),
                    &na::translation(&bis.borrow().anchor2_pos()).translate(&na::orig()),
                    &Color::new_rgb(255, 0, 0)
                );
            }
        }
    }
}

pub fn draw_line(window: &mut graphics::RenderWindow, v1: &Pnt2<f32>, v2: &Pnt2<f32>, color: &Color) {
    let mut vertices = VertexArray::new().unwrap();

    vertices.append(&Vertex::new(
            &Vector2f::new(v1.x * DRAW_SCALE, v1.y * DRAW_SCALE),
            color,
            &Vector2f::new(0.0, 0.0)));

    vertices.append(&Vertex::new(
            &Vector2f::new(v2.x * DRAW_SCALE, v2.y * DRAW_SCALE),
            color,
            &Vector2f::new(0.0, 0.0)));

    vertices.set_primitive_type(graphics::Lines);

    window.draw(&vertices);
}
