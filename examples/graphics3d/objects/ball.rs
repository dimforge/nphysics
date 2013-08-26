use kiss3d::window::Window;
use kiss3d::object;
use nalgebra::traits::transformation::Transformation;
use nalgebra::vec::Vec3;
use nphysics::aliases::dim3;
use engine::SceneNode;

struct Ball {
    priv color:      Vec3<f32>,
    priv base_color: Vec3<f32>,
    priv delta:      dim3::Transform3d<f64>,
    priv gfx:        @mut object::Object,
    priv body:       @mut dim3::RigidBody3d<f64>
}

impl Ball {
    pub fn new(body:   @mut dim3::RigidBody3d<f64>,
               delta:  dim3::Transform3d<f64>,
               radius: f64,
               color:  Vec3<f32>,
               window: @mut Window) -> Ball {
        let mut res = Ball {
            color:      color,
            base_color: color,
            delta:      delta,
            gfx:        window.add_sphere(radius as f32).set_color(color.x, color.y, color.z),
            body:       body
        };

        res.update();

        res
    }
}

impl SceneNode for Ball {
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
