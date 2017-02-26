use kiss3d::window::Window;
use kiss3d::scene::SceneNode;
use na::{Vector3, Point3, Isometry3};
use ncollide::procedural::TriMesh;
use nphysics3d::object::WorldObject;
use objects::node;

pub struct Convex {
    color:      Point3<f32>,
    base_color: Point3<f32>,
    delta:      Isometry3<f32>,
    gfx:        SceneNode,
    body:       WorldObject<f32>
}

impl Convex {
    pub fn new(body:   WorldObject<f32>,
               delta:  Isometry3<f32>,
               convex: &TriMesh<Point3<f32>>,
               color:  Point3<f32>,
               window: &mut Window)
               -> Convex {
        let t         = body.borrow().position().clone();
        let is_sensor = body.is_sensor();

        let mut res = Convex {
            color:      color,
            base_color: color,
            delta:      delta,
            gfx:        window.add_trimesh(convex.clone(), Vector3::from_element(1.0)),
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
