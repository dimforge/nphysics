use crate::objects::node;
use kiss3d::resource;
use kiss3d::scene::SceneNode;
use kiss3d::window::Window;
use na::{self, Isometry3, Point3, RealField, Vector3};
use ncollide::shape::TriMesh;
use nphysics::math::Isometry;
use nphysics::object::{ColliderAnchor, DefaultColliderHandle, DefaultColliderSet};
use std::cell::RefCell;
use std::rc::Rc;

pub struct Mesh {
    color: Point3<f32>,
    base_color: Point3<f32>,
    delta: Isometry3<f32>,
    gfx: SceneNode,
    collider: DefaultColliderHandle,
}

impl Mesh {
    pub fn new<N: RealField>(
        collider: DefaultColliderHandle,
        colliders: &DefaultColliderSet<N>,
        delta: Isometry3<f32>,
        vertices: Vec<Point3<f32>>,
        indices: Vec<Point3<u32>>,
        color: Point3<f32>,
        window: &mut Window,
    ) -> Mesh {
        let vs = vertices;
        let is = indices.into_iter().map(na::convert).collect();

        let mesh = resource::Mesh::new(vs, is, None, None, false);

        let mut res = Mesh {
            color,
            base_color: color,
            delta,
            gfx: window.add_mesh(Rc::new(RefCell::new(mesh)), Vector3::from_element(1.0)),
            collider,
        };

        if colliders
            .get(collider)
            .unwrap()
            .query_type()
            .is_proximity_query()
        {
            res.gfx.set_surface_rendering_activation(false);
            res.gfx.set_lines_width(1.0);
        }

        let pos: Isometry<f64> =
            na::convert_unchecked(*colliders.get(collider).unwrap().position());
        let pos: Isometry<f32> = na::convert(pos);

        res.gfx.enable_backface_culling(false);
        res.gfx.set_color(color.x, color.y, color.z);
        res.gfx.set_local_transformation(pos * res.delta);
        res.update(colliders);

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

    pub fn update<N: RealField>(&mut self, colliders: &DefaultColliderSet<N>) {
        node::update_scene_node(
            &mut self.gfx,
            colliders,
            self.collider,
            &self.color,
            &self.delta,
        );

        // Update if some deformation occurred.
        // FIXME: don't update if it did not move.
        if let Some(c) = colliders.get(self.collider) {
            if let ColliderAnchor::OnDeformableBody { .. } = c.anchor() {
                let shape = c.shape().as_shape::<TriMesh<N>>().unwrap();
                let vtx = shape.points();

                self.gfx.modify_vertices(&mut |vertices| {
                    for (v, new_v) in vertices.iter_mut().zip(vtx.iter()) {
                        *v = na::convert::<Point3<f64>, Point3<f32>>(na::convert_unchecked(*new_v));
                    }
                });
                self.gfx.recompute_normals();
            }
        }
    }

    pub fn scene_node(&self) -> &SceneNode {
        &self.gfx
    }

    pub fn scene_node_mut(&mut self) -> &mut SceneNode {
        &mut self.gfx
    }

    pub fn object(&self) -> DefaultColliderHandle {
        self.collider
    }
}
