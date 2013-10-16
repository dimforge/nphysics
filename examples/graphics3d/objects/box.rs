use kiss3d::window;
use kiss3d::object::Object;
use nalgebra::na::{Vec3, Transformation};
use nalgebra::na;
use nphysics::aliases::dim3;
use engine::SceneNode;

struct Box {
    priv color:      Vec3<f32>,
    priv base_color: Vec3<f32>,
    priv delta:      dim3::Transform3d<f64>,
    priv gfx:        Object,
    priv body:       @mut dim3::Body3d<f64>,
}

impl Box {
    pub fn new(body:   @mut dim3::Body3d<f64>,
               delta:  dim3::Transform3d<f64>,
               rx:     f64,
               ry:     f64,
               rz:     f64,
               color:  Vec3<f32>,
                       window: &mut window::Window) -> Box {
        let gx = rx as f32 * 2.0;
        let gy = ry as f32 * 2.0;
        let gz = rz as f32 * 2.0;

        let mut res = Box {
            color:      color,
            base_color: color,
            delta:      delta,
            gfx:        window.add_cube(gx, gy, gz),
            body:       body
        };

        res.gfx.set_color(color.x, color.y, color.z);
        res.update();

        res
    }
}

impl SceneNode for Box {
    fn select(&mut self) {
        self.color = Vec3::x();
    }

    fn unselect(&mut self) {
        self.color = self.base_color;
    }

    fn update(&mut self) {
        let rb = self.body.to_rigid_body_or_fail();

        if rb.is_active() {
                self.gfx.set_transformation(na::transformation(rb) * self.delta);
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
