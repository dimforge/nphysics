use std::num::One;
use kiss3d::window;
use kiss3d::object;
use nalgebra::traits::transformation::Transformation;
use nalgebra::traits::rotation::Rotation;
use nalgebra::vec::Vec3;
use nphysics::aliases::dim3;
use engine::SceneNode;

struct Cylinder {
    priv color:      Vec3<f32>,
    priv base_color: Vec3<f32>,
    priv delta:      dim3::Transform3d<f64>,
    priv gfx:        @mut object::Object,
    priv body:       @mut dim3::RigidBody3d<f64>,
}

impl Cylinder {
    pub fn new(body:   @mut dim3::RigidBody3d<f64>,
               delta:  dim3::Transform3d<f64>,
               r:     f64,
               h:     f64,
               color:  Vec3<f32>,
               window: @mut window::Window) -> Cylinder {
        let mut realign: dim3::Transform3d<f64> = One::one();
        let _frac_pi_2: f64 = Real::frac_pi_2();
        realign.rotate_by(&Vec3::new(0.0f64, 0.0, -_frac_pi_2));

        let mut res = Cylinder {
            color:      color,
            base_color: color,
            delta: delta * realign,
            gfx:   window.add_cylinder(h as f32, r as f32).set_color(color.x, color.y, color.z),
            body:  body
        };

        res.update();

        res
    }
}

impl SceneNode for Cylinder {
    fn select(&mut self) {
        self.color = Vec3::x();
    }

    fn unselect(&mut self) {
        self.color = self.base_color;
    }

    fn update(&mut self) {
        if self.body.is_active() {
            {
                let gfx_transform = self.gfx.transformation();
                *gfx_transform    = self.body.transformation() * self.delta;
            }

            self.gfx.set_color(self.color.x, self.color.y, self.color.z);
        }
        else {
            self.gfx.set_color(self.color.x * 0.25, self.color.y * 0.25, self.color.z * 0.25);
        }
    }
}
