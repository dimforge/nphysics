use kiss3d::resource;
use kiss3d::scene::SceneNode;
use kiss3d::window::Window;
use na::{self, Isometry3, Point3, Vector3};
use nphysics3d::object::{BodyHandle, DeformableVolume};
use nphysics3d::world::World;
use objects::node;
use std::cell::RefCell;
use std::rc::Rc;

pub struct DeformableMesh {
    color: Point3<f32>,
    base_color: Point3<f32>,
    gfx: SceneNode,
    body: BodyHandle,
}

impl DeformableMesh {
    pub fn new(
        body: BodyHandle,
        world: &World<f32>,
        vertices: Vec<Point3<f32>>,
        indices: Vec<Point3<u16>>,
        color: Point3<f32>,
        window: &mut Window,
    ) -> DeformableMesh {
        let vs = vertices;

        let mesh = resource::Mesh::new(vs, indices, None, None, false);

        let mut res = DeformableMesh {
            color,
            base_color: color,
            gfx: window.add_mesh(Rc::new(RefCell::new(mesh)), Vector3::from_element(1.0)),
            body,
        };

//        {
//            res.gfx.set_surface_rendering_activation(false);
//            res.gfx.set_lines_width(1.0);
//        }

        res.gfx.enable_backface_culling(false);
        res.gfx.set_color(color.x, color.y, color.z);
        res.update(world);

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

    pub fn update(&mut self, world: &World<f32>) {
        let b = world.body(self.body);
        let volume = b.downcast_ref::<DeformableVolume<f32>>().unwrap();
        let coords = volume.positions();

        self.gfx.modify_vertices(&mut |vertices| {
            for (v, pos) in vertices.iter_mut().zip(coords.as_slice().chunks(3)) {
                v.x = pos[0];
                v.y = pos[1];
                v.z = pos[2];
            }
        });
    }

    pub fn scene_node(&self) -> &SceneNode {
        &self.gfx
    }

    pub fn scene_node_mut(&mut self) -> &mut SceneNode {
        &mut self.gfx
    }
}
