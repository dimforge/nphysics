use kiss3d::window;
use kiss3d::object::Object;
use nalgebra::na::Vec3;
use nphysics::object::Body;
use engine::SceneNode;

pub struct Plane {
    priv gfx:  Object,
    priv body: @mut Body,
}

impl Plane {
    pub fn new(body:   @mut Body,
               pos:    &Vec3<f32>,
               normal: &Vec3<f32>,
               color:  Vec3<f32>,
               window: &mut window::Window) -> Plane {

        let mut res = Plane {
            gfx:  window.add_quad(100.0, 100.0, 10, 10, false),
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
