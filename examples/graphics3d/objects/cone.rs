use std::num::One;
use kiss3d::window;
use kiss3d::object::Object;
use nalgebra::traits::transformation::Transformation;
use nalgebra::traits::rotation::Rotation;
use nalgebra::vec::Vec3;
use nphysics::aliases::dim3;
use engine::SceneNode;

struct Cone {
    priv color:      Vec3<f32>,
    priv base_color: Vec3<f32>,
    priv delta:      dim3::Transform3d<f64>,
    priv gfx:        Object,
    priv body:       @mut dim3::Body3d<f64>,
}

impl Cone {
    pub fn new(body:   @mut dim3::Body3d<f64>,
               delta:  dim3::Transform3d<f64>,
               r:      f64,
               h:      f64,
               color:  Vec3<f32>,
               window: &mut window::Window) -> Cone {
        let mut realign: dim3::Transform3d<f64> = One::one();
        let _frac_pi_2: f64 = Real::frac_pi_2();
        realign.rotate_by(&Vec3::new(0.0, 0.0, -_frac_pi_2));

        let mut res = Cone {
            color:      color,
            base_color: color,
            delta:      delta * realign,
            gfx:        window.add_cone(h as f32, r as f32),
            body:       body
        };

        res.gfx.set_color(color.x, color.y, color.z);
        res.update();

        res
    }
}

impl SceneNode for Cone {
    fn select(&mut self) {
        self.color = Vec3::x();
    }

    fn unselect(&mut self) {
        self.color = self.base_color;
    }

    fn update(&mut self) {
        let rb = self.body.to_rigid_body_or_fail();
        if rb.is_active() {
            self.gfx.set_transformation(rb.transformation() * self.delta);
            self.gfx.set_color(self.color.x, self.color.y, self.color.z);
        }
        else {
            self.gfx.set_color(self.color.x * 0.25, self.color.y * 0.25, self.color.z * 0.25);
        }
    }

    fn object<'r>(&'r self) -> &'r Object {
        &self.gfx
    }
}
