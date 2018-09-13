use kiss3d::resource;
use kiss3d::scene::SceneNode;
use kiss3d::window::Window;
use na::{self, Isometry3, Point3, Vector3};
use ncollide3d::shape::TriMesh;
use nphysics3d::object::{ColliderHandle, ColliderAnchor};
use nphysics3d::world::World;
use objects::node;
use std::cell::RefCell;
use std::rc::Rc;

pub struct Mesh {
    color: Point3<f32>,
    base_color: Point3<f32>,
    delta: Isometry3<f32>,
    gfx: SceneNode,
    collider: ColliderHandle,
}

impl Mesh {
    pub fn new(
        collider: ColliderHandle,
        world: &World<f32>,
        delta: Isometry3<f32>,
        vertices: Vec<Point3<f32>>,
        indices: Vec<Point3<u32>>,
        color: Point3<f32>,
        window: &mut Window,
    ) -> Mesh {
        let vs = vertices;
        let is = indices.into_iter().map(|p| na::convert(p)).collect();

        let mesh = resource::Mesh::new(vs, is, None, None, false);

        let mut res = Mesh {
            color: color,
            base_color: color,
            delta: delta,
            gfx: window.add_mesh(Rc::new(RefCell::new(mesh)), Vector3::from_element(1.0)),
            collider: collider,
        };

        if world
            .collider(collider)
            .unwrap()
            .query_type()
            .is_proximity_query()
            {
                res.gfx.set_surface_rendering_activation(false);
                res.gfx.set_lines_width(1.0);
            }

        res.gfx.enable_backface_culling(false);
        res.gfx.set_color(color.x, color.y, color.z);
        res.gfx
            .set_local_transformation(world.collider(collider).unwrap().position() * res.delta);
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
        node::update_scene_node(
            &mut self.gfx,
            world,
            self.collider,
            &self.color,
            &self.delta,
        );

        // Update if some deformation occurred.
        // FIXME: don't update if it did not move.
        if let Some(c) = world.collider(self.collider) {
            if let ColliderAnchor::OnDeformableBody { .. } = c.data().anchor() {
                let shape = c.shape().as_shape::<TriMesh<f32>>().unwrap();
                let vtx = shape.vertices();

                self.gfx.modify_vertices(&mut |vertices| {
                    for (v, new_v) in vertices.iter_mut().zip(vtx.iter()) {
                        *v = *new_v
                    }
                });
            }
        }
    }

    pub fn scene_node(&self) -> &SceneNode {
        &self.gfx
    }

    pub fn scene_node_mut(&mut self) -> &mut SceneNode {
        &mut self.gfx
    }

    pub fn object(&self) -> ColliderHandle {
        self.collider
    }
}
