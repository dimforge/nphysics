use crate::objects::node::GraphicsNode;
use kiss3d::window::Window;
use na::{Point3, RealField, Vector3};
use nphysics::math::{Point, Vector};
use salva::object::{Boundary, Fluid as SalvaFluid};

#[derive(Copy, Clone, Debug)]
pub enum FluidRenderingMode {
    VelocityColor { min: f32, max: f32 },
    StaticColor,
}

pub struct Fluid {
    radius: f32,
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
            radius,
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

    fn update(
        &mut self,
        centers: &[Point<f32>],
        velocities: &[Vector<f32>],
        mode: FluidRenderingMode,
    ) {
        if centers.len() > self.balls_gfx.len() {
            for _ in 0..centers.len() - self.balls_gfx.len() {
                #[cfg(feature = "dim2")]
                let ball_gfx = self.gfx.add_circle(self.radius);
                #[cfg(feature = "dim3")]
                let ball_gfx = self.gfx.add_sphere(self.radius);
                self.balls_gfx.push(ball_gfx);
            }
        }

        for ball_gfx in &mut self.balls_gfx[centers.len()..] {
            ball_gfx.set_visible(false);
        }

        for (i, (pt, ball)) in centers.iter().zip(self.balls_gfx.iter_mut()).enumerate() {
            ball.set_visible(true);
            ball.set_local_translation(pt.coords.into());

            let color = match mode {
                FluidRenderingMode::StaticColor => self.base_color,
                FluidRenderingMode::VelocityColor { min, max } => {
                    let start = self.base_color.coords;
                    let end = Vector3::new(1.0, 0.0, 0.0);
                    let vel = velocities[i];
                    let t = (vel.norm() - min) / (max - min);
                    start.lerp(&end, na::clamp(t, 0.0, 1.0)).into()
                }
            };

            ball.set_color(color.x, color.y, color.z);
        }
    }

    pub fn update_with_boundary(&mut self, boundary: &Boundary<N>) {
        self.update(&boundary.positions, &[], FluidRenderingMode::StaticColor)
    }

    pub fn update_with_fluid(
        &mut self,
        fluid: &SalvaFluid<f32>,
        rendering_mode: FluidRenderingMode,
    ) {
        self.update(&fluid.positions, &fluid.velocities, rendering_mode)
    }

    pub fn scene_node(&self) -> &GraphicsNode {
        &self.gfx
    }

    pub fn scene_node_mut(&mut self) -> &mut GraphicsNode {
        &mut self.gfx
    }
}
