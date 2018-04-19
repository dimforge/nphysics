use sfml::graphics;
use sfml::graphics::{Color, RenderTarget, Vertex, VertexArray};
use sfml::system::Vector2f;
use na::Point2;
use ncollide2d::query::ContactKinematic;
use nphysics2d::world::World;

pub static DRAW_SCALE: f32 = 20.0;

pub fn draw_colls(window: &mut graphics::RenderWindow, world: &World<f32>) {
    for (_, _, manifold) in world.collision_world().contact_manifolds() {
        for (i, c) in manifold.contacts().iter().enumerate() {
            let color = if i == manifold.deepest_contact_id() {
                Color::new_rgb(255, 0, 0)
            } else {
                Color::new_rgb(0, 0, 255)
            };
            draw_line(window, &c.contact.world1, &c.contact.world2, &color);
        }
    }
}

pub fn draw_line(
    window: &mut graphics::RenderWindow,
    v1: &Point2<f32>,
    v2: &Point2<f32>,
    color: &Color,
) {
    let mut vertices = VertexArray::new().unwrap();

    vertices.append(&Vertex::new(
        &Vector2f::new(v1.x * DRAW_SCALE, v1.y * DRAW_SCALE),
        color,
        &Vector2f::new(0.0, 0.0),
    ));

    vertices.append(&Vertex::new(
        &Vector2f::new(v2.x * DRAW_SCALE, v2.y * DRAW_SCALE),
        color,
        &Vector2f::new(0.0, 0.0),
    ));

    vertices.set_primitive_type(graphics::Lines);

    window.draw(&vertices);
}
