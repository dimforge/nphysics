use std::sync::Arc;
use sfml::graphics;
use sfml::graphics::Color;
use na::{Isometry2, Point2, Point3};
use nphysics2d::world::World;
use nphysics2d::object::ColliderHandle;
use draw_helper::draw_line;

pub struct Lines {
    color: Point3<u8>,
    base_color: Point3<u8>,
    delta: Isometry2<f32>,
    collider: ColliderHandle,
    indices: Arc<Vec<Point2<usize>>>,
    vertices: Arc<Vec<Point2<f32>>>,
}

impl Lines {
    pub fn new(
        collider: ColliderHandle,
        world: &World<f32>,
        delta: Isometry2<f32>,
        vertices: Arc<Vec<Point2<f32>>>,
        indices: Arc<Vec<Point2<usize>>>,
        color: Point3<u8>,
    ) -> Lines {
        Lines {
            color: color,
            base_color: color,
            delta: delta,
            collider: collider,
            vertices: vertices,
            indices: indices,
        }
    }
}

impl Lines {
    pub fn collider(&self) -> ColliderHandle {
        self.collider
    }

    pub fn update(&mut self, _: &World<f32>) {}

    pub fn draw(&self, rw: &mut graphics::RenderWindow, world: &World<f32>) {
        let co = world.collider(self.collider).unwrap();
        let active = world.body(co.data().body()).is_active();

        let pos = co.position() * self.delta;

        let color = if active {
            Color::new_rgb(self.color.x, self.color.y, self.color.z)
        } else {
            Color::new_rgb(self.color.x / 4, self.color.y / 4, self.color.z / 4)
        };

        let vs = &*self.vertices;

        for is in self.indices.iter() {
            let gsv0 = pos * vs[is.x];
            let gsv1 = pos * vs[is.y];
            draw_line(rw, &gsv0, &gsv1, &color);
        }
    }

    pub fn set_color(&mut self, color: Point3<u8>) {
        self.color = color;
        self.base_color = color;
    }

    pub fn select(&mut self) {
        self.color = Point3::new(200, 0, 0);
    }

    pub fn unselect(&mut self) {
        self.color = self.base_color;
    }
}
