use kiss3d::resource;
use kiss3d::scene::SceneNode2;
use kiss3d::window::Window;
use na::{self, Isometry2, Point2, Point3, Vector3};
use nphysics2d::object::ColliderHandle;
use nphysics2d::world::World;
use objects::node;

pub struct Polyline {
    color: Point3<f32>,
    base_color: Point3<f32>,
    delta: Isometry2<f32>,
    vertices: Vec<Point2<f32>>,
    collider: ColliderHandle,
}

impl Polyline {
    pub fn new(
        collider: ColliderHandle,
        world: &World<f32>,
        delta: Isometry2<f32>,
        vertices: Vec<Point2<f32>>,
        color: Point3<f32>,
        window: &mut Window,
    ) -> Polyline {
        let mut res = Polyline {
            color,
            base_color: color,
            delta,
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

    pub fn update(&mut self, world: &World<f32>) {}

    pub fn object(&self) -> ColliderHandle {
        self.collider
    }

    pub fn draw(&mut self, window: &mut Window) {
        for vtx in self.vertices.windows(2) {
            window.draw_planar_line(&vtx[0], &vtx[1], &self.color)
        }
    }
}
