use kiss3d::window;
use kiss3d::object::Object;
use nalgebra::na::Vec3;
use nphysics::aliases::dim3;
use engine::SceneNode;

struct Plane {
    priv gfx:  Object,
    priv body: @mut dim3::Body3d<f64>,
}

impl Plane {
    pub fn new(body:   @mut dim3::Body3d<f64>,
               pos:    &Vec3<f64>,
               normal: &Vec3<f64>,
               color:  Vec3<f32>,
               window: &mut window::Window) -> Plane {

        let mut res = Plane {
            gfx:  window.add_quad(100.0, 100.0, 10, 10),
            body: body
        };

        res.gfx.set_color(color.x, color.y, color.z);
        res.gfx.look_at_z(pos, normal, &Vec3::new(1.0, 0.0, 0.0));

        res.update();

        res
    }
}

impl SceneNode for Plane {
    fn select(&mut self) {
    }

    fn unselect(&mut self) {
    }

    fn update(&mut self) {
        // FIXME: atm we assume the plane does not move
    }

    fn object<'r>(&'r self) -> &'r Object {
        &self.gfx
    }
}
