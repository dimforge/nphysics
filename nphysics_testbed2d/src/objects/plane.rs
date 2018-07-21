use kiss3d::window::Window;
use na::{Point2, Point3, Unit, Vector2};
use nphysics2d::object::ColliderHandle;
use nphysics2d::world::World;

pub struct Plane {
    color: Point3<f32>,
    base_color: Point3<f32>,
    position: Point2<f32>,
    normal: Unit<Vector2<f32>>,
    collider: ColliderHandle,
}

impl Plane {
    pub fn new(
        collider: ColliderHandle,
        world: &World<f32>,
        position: &Point2<f32>,
        normal: &Unit<Vector2<f32>>,
        color: Point3<f32>,
        _: &mut Window,
    ) -> Plane {
        let mut res = Plane {
            color,
            base_color: color,
            position: *position,
            normal: *normal,
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
        let orth = Vector2::new(-self.normal.y, self.normal.x);
        window.draw_planar_line(
            &(self.position - orth * 50.0),
            &(self.position + orth * 50.0),
            &self.color,
        );
    }
}
