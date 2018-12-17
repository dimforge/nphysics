use kiss3d::window::Window;
use na::{Isometry2, Point2, Point3, DVector, Vector2};
use ncollide2d::shape;
use nphysics2d::object::{ColliderHandle, ColliderAnchor};
use nphysics2d::world::World;

pub struct HeightField {
    color: Point3<f32>,
    base_color: Point3<f32>,
    vertices: Vec<Point2<f32>>,
    collider: ColliderHandle,
}

impl HeightField {
    pub fn new(
        collider: ColliderHandle,
        world: &World<f32>,
        _: Isometry2<f32>,
        heightfield: &shape::HeightField<f32>,
        color: Point3<f32>,
        _: &mut Window,
    ) -> HeightField {
        let heights = heightfield.heights();
        let scale = heightfield.scale();
        let mut vertices = Vec::new();
        let step = 1.0 / (heights.len() as f32 - 1.0);

        for (i, ys) in heights.as_slice().windows(2).enumerate() {
            if !heightfield.is_segment_removed(i) {
                let a = Point2::new((i as f32 * step - 0.5) * scale.x, ys[0] * scale.y);
                let b = Point2::new(((i + 1) as f32 * step - 0.5) * scale.x, ys[1] * scale.y);
                vertices.push(a);
                vertices.push(b);
            }
        }

        let mut res = HeightField {
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

    pub fn update(&mut self, world: &World<f32>) {
    }

    pub fn object(&self) -> ColliderHandle {
        self.collider
    }

    pub fn draw(&mut self, window: &mut Window) {
        for vtx in self.vertices.chunks(2) {
            window.draw_planar_line(&vtx[0], &vtx[1], &self.color)
        }
    }
}
