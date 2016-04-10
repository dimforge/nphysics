use kiss3d::window;
use kiss3d::scene::SceneNode;
use na::{Pnt3, Iso3};
use nphysics3d::object::WorldObject;
use objects::node;

pub struct Box {
    color:      Pnt3<f32>,
    base_color: Pnt3<f32>,
    delta:      Iso3<f32>,
    gfx:        SceneNode,
    body:       WorldObject<f32>,
}

impl Box {
    pub fn new(body:   WorldObject<f32>,
               delta:  Iso3<f32>,
               rx:     f32,
               ry:     f32,
               rz:     f32,
               color:  Pnt3<f32>,
               window: &mut window::Window) -> Box {
        let gx = rx * 2.0;
        let gy = ry * 2.0;
        let gz = rz * 2.0;
        let t         = body.borrow().position().clone();
        let is_sensor = body.is_sensor();

        let mut res = Box {
            color:      color,
            base_color: color,
            delta:      delta,
            gfx:        window.add_cube(gx, gy, gz),
            body:       body
        };

        if is_sensor {
            res.gfx.set_surface_rendering_activation(false);
            res.gfx.set_lines_width(1.0);
        }

        res.gfx.set_color(color.x, color.y, color.z);
        res.gfx.set_local_transformation(t * res.delta);
        res.update();

        res
    }

    pub fn select(&mut self) {
        self.color = Pnt3::new(1.0, 0.0, 0.0);
    }

    pub fn unselect(&mut self) {
        self.color = self.base_color;
    }

    pub fn set_color(&mut self, color: Pnt3<f32>) {
        self.gfx.set_color(color.x, color.y, color.z);
        self.color = color;
        self.base_color = color;
    }

    pub fn update(&mut self) {
        node::update_scene_node(&mut self.gfx, &self.body, &self.color, &self.delta);
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
