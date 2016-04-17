use na;
use kiss3d::window;
use kiss3d::scene::SceneNode;
use na::{Point3, Vector3};
use nphysics3d::object::WorldObject;

pub struct Plane {
    gfx:  SceneNode,
    body: WorldObject<f32>
}

impl Plane {
    pub fn new(body:         WorldObject<f32>,
               world_pos:    &Point3<f32>,
               world_normal: &Vector3<f32>,
               color:        Point3<f32>,
               window:       &mut window::Window) -> Plane {
        let is_sensor = body.is_sensor();

        let mut res = Plane {
            gfx:  window.add_quad(100.0, 100.0, 10, 10),
            body: body
        };

        if is_sensor {
            res.gfx.set_surface_rendering_activation(false);
            res.gfx.set_lines_width(1.0);
        }

        res.gfx.set_color(color.x, color.y, color.z);

        let up;

        if na::is_zero(&world_normal.z) && na::is_zero(&world_normal.y) {
            up = Vector3::z();
        }
        else {
            up = Vector3::x();
        }

        res.gfx.reorient(world_pos, &(*world_pos + *world_normal), &up);

        res.update();

        res
    }

    pub fn select(&mut self) {
    }

    pub fn unselect(&mut self) {
    }

    pub fn update(&mut self) {
        // FIXME: atm we assume the plane does not move
    }

    pub fn set_color(&mut self, color: Point3<f32>) {
        self.gfx.set_color(color.x, color.y, color.z);
    }

    pub fn scene_node(&self) -> &SceneNode {
        &self.gfx
    }

    pub fn scene_node_mut(&mut self) -> &mut SceneNode {
        &mut self.gfx
    }

    pub fn object(&self) -> &WorldObject<f32> {
        &self.body
    }
}
