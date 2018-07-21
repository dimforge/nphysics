use kiss3d::window::Window;
use na::{Isometry2, Point2, Point3};
use nphysics2d::object::ColliderHandle;
use nphysics2d::world::World;

pub struct Polyline {
    color: Point3<f32>,
    base_color: Point3<f32>,
    vertices: Vec<Point2<f32>>,
    collider: ColliderHandle,
}

impl Polyline {
    pub fn new(
        collider: ColliderHandle,
        world: &World<f32>,
        _: Isometry2<f32>,
        vertices: Vec<Point2<f32>>,
        color: Point3<f32>,
        _: &mut Window,
    ) -> Polyline {
        let mut res = Polyline {
            color,
            base_color: color,
            vertices,
            collider,
        };

        res.update(world);
        res
    }

    pub fn select(&mut self) {
        self.color = Point3::new(1.0, 0.0, 0.0);
    }

    pub fn unselect(&mut self) {
        self.color = self.base_color;
    }

    pub fn set_color(&mut self, color: Point3<f32>) {
        self.color = color;
        self.base_color = color;
    }

    pub fn update(&mut self, _: &World<f32>) {}

    pub fn object(&self) -> ColliderHandle {
        self.collider
    }

    pub fn draw(&mut self, window: &mut Window) {
        for vtx in self.vertices.windows(2) {
            window.draw_planar_line(&vtx[0], &vtx[1], &self.color)
        }
    }
}
