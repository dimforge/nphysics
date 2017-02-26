use std::rc::Rc;
use std::cell::RefCell;
use kiss3d::window::Window;
use kiss3d::scene::SceneNode;
use na::{Vector3, Point3, Isometry3};
use na;
use nphysics::object::WorldObject;
use ncollide::procedural;
use ncollide::parametric;

pub struct ParametricSurface {
    color:      Point3<f32>,
    base_color: Point3<f32>,
    delta:      Isometry3<f32>,
    gfx:        SceneNode,
    body:       WorldObject<f32>
}

impl ParametricSurface {
    pub fn new<S: parametric::ParametricSurface>(body:    WorldObject<f32>,
                                                 delta:   Isometry3<f32>,
                                                 surface: &S,
                                                 color:   Point3<f32>,
                                                 window:  &mut Window)
                                                 -> ParametricSurface {
        let t     = na::transformation(body.borrow().deref());
        let param = procedural::parametric_surface_uniform(surface, 100, 100);

        let mut res = ParametricSurface {
            color:      color,
            base_color: color,
            delta:      delta,
            gfx:        window.add_trimesh(param, Vector3::from_element(1.0)),
            body:       body
        };

        if body.is_sensor() {
            res.gfx.set_surface_rendering_activation(false);
            res.gfx.set_lines_width(1.0);
        }

        res.gfx.set_color(color.x, color.y, color.z);
        res.gfx.set_local_transformation(t * res.delta);
        res.gfx.enable_backface_culling(false);
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
        res.gfx.set_color(color.x, color.y, color.z);
        self.color = color;
        self.base_color = color;
    }

    pub fn update(&mut self) {
        let rb = self.body.borrow();

        if rb.is_active() {
            self.gfx.set_local_transformation(na::transformation(rb.deref()) * self.delta);
            self.gfx.set_color(self.color.x, self.color.y, self.color.z);
        }
        else {
            self.gfx.set_color(self.color.x * 0.25, self.color.y * 0.25, self.color.z * 0.25);
        }
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
