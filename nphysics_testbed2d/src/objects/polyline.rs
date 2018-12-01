use kiss3d::window::Window;
use na::{Isometry2, Point2, Point3};
use ncollide2d::shape;
use nphysics2d::object::{ColliderHandle, ColliderAnchor};
use nphysics2d::world::World;

pub struct Polyline {
    color: Point3<f32>,
    base_color: Point3<f32>,
    vertices: Vec<Point2<f32>>,
    indices: Vec<Point2<usize>>,
    collider: ColliderHandle,
}

impl Polyline {
    pub fn new(
        collider: ColliderHandle,
        world: &World<f32>,
        _: Isometry2<f32>,
        vertices: Vec<Point2<f32>>,
        indices: Vec<Point2<usize>>,
        color: Point3<f32>,
        _: &mut Window,
    ) -> Polyline {
        let mut res = Polyline {
            color,
            base_color: color,
            vertices,
            indices,
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

    pub fn update(&mut self, world: &World<f32>) {
        // Update if some deformation occurred.
        // FIXME: don't update if it did not move.
        if let Some(c) = world.collider(self.collider) {
            if let ColliderAnchor::OnDeformableBody { .. } = c.data().anchor() {
                let shape = c.shape().as_shape::<shape::Polyline<f32>>().unwrap();
                self.vertices = shape.points().to_vec();
                self.indices.clear();

                for e in shape.edges() {
                    self.indices.push(e.indices);
                }
            }
        }
    }

    pub fn object(&self) -> ColliderHandle {
        self.collider
    }

    pub fn draw(&mut self, window: &mut Window) {
        for idx in &self.indices {
            window.draw_planar_line(&self.vertices[idx.x], &self.vertices[idx.y], &self.color)
        }
    }
}
