use crate::objects::node::{self, GraphicsNode};
use kiss3d::window::Window;
use na::Point3;
use ncollide::shape;
use nphysics::math::{Isometry, Point};
use nphysics::object::{ColliderAnchor, DefaultColliderHandle, DefaultColliderSet};

pub struct Fluid {
    color: Point3<f32>,
    base_color: Point3<f32>,
    gfx: GraphicsNode,
    balls_gfx: Vec<GraphicsNode>,
}

impl Fluid {
    pub fn new(
        radius: f32,
        centers: &[Point<f32>],
        color: Point3<f32>,
        window: &mut Window,
    ) -> Fluid {
        #[cfg(feature = "dim2")]
        let mut gfx = window.add_planar_group();
        #[cfg(feature = "dim3")]
        let mut gfx = window.add_group();

        let mut balls_gfx = Vec::new();

        for c in centers {
            #[cfg(feature = "dim2")]
            let mut ball_gfx = gfx.add_circle(radius);
            #[cfg(feature = "dim3")]
            let mut ball_gfx = gfx.add_sphere(radius);
            ball_gfx.set_local_translation(c.coords.into());
            balls_gfx.push(ball_gfx);
        }

        let mut res = Fluid {
            color,
            base_color: color,
            gfx,
            balls_gfx,
        };

        res.gfx.set_color(color.x, color.y, color.z);
        res
    }

    pub fn select(&mut self) {
        self.color = Point3::new(1.0, 0.0, 0.0);
    }

    pub fn unselect(&mut self) {
        self.color = self.base_color;
    }

    pub fn set_color(&mut self, color: Point3<f32>) {
        self.gfx.set_color(color.x, color.y, color.z);
        self.color = color;
        self.base_color = color;
    }

    pub fn update(&mut self, centers: &[Point<f32>]) {
        for (pt, ball) in centers.iter().zip(self.balls_gfx.iter_mut()) {
            ball.set_local_translation(pt.coords.into())
        }
    }

    pub fn scene_node(&self) -> &GraphicsNode {
        &self.gfx
    }

    pub fn scene_node_mut(&mut self) -> &mut GraphicsNode {
        &mut self.gfx
    }
}
