use kiss3d::window::Window;
use na::{self, Point3};
#[cfg(feature = "dim2")]
use nphysics::math::Point;
use nphysics::math::Isometry;
#[cfg(feature = "dim3")]
use nphysics::math::Vector;
use nphysics::object::{DefaultColliderHandle, DefaultColliderSet};
use ncollide::shape;
#[cfg(feature = "dim3")]
use ncollide::transformation::ToTriMesh;
#[cfg(feature = "dim3")]
use crate::objects::node::{self, GraphicsNode};

pub struct HeightField {
    color: Point3<f32>,
    base_color: Point3<f32>,
    #[cfg(feature = "dim3")]
    delta: Isometry<f32>,
    #[cfg(feature = "dim2")]
    vertices: Vec<Point<f32>>,
    #[cfg(feature = "dim3")]
    gfx: GraphicsNode,
    collider: DefaultColliderHandle,
}

impl HeightField {
    #[cfg(feature = "dim2")]
    pub fn new(
        collider: DefaultColliderHandle,
        colliders: &DefaultColliderSet<f32>,
        _: Isometry<f32>,
        heightfield: &shape::HeightField<f32>,
        color: Point3<f32>,
        _: &mut Window,
    ) -> HeightField {
        let mut vertices = Vec::new();

        for seg in heightfield.segments() {
            vertices.push(*seg.a());
            vertices.push(*seg.b());
        }

        let mut res = HeightField {
            color,
            base_color: color,
            vertices,
            collider,
        };

        res.update(colliders);
        res
    }

    #[cfg(feature = "dim3")]
    pub fn new(
        collider: DefaultColliderHandle,
        colliders: &DefaultColliderSet<f32>,
        delta: Isometry<f32>,
        heightfield: &shape::HeightField<f32>,
        color: Point3<f32>,
        window: &mut Window,
    ) -> HeightField {
        let mesh = heightfield.to_trimesh(());

        let mut res = HeightField {
            color: color,
            base_color: color,
            delta: delta,
            gfx: window.add_trimesh(mesh, Vector::repeat(1.0)),
            collider: collider,
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

        res.gfx.enable_backface_culling(false);
        res.gfx.set_color(color.x, color.y, color.z);
        res.gfx
            .set_local_transformation(colliders.get(collider).unwrap().position() * res.delta);
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
        #[cfg(feature = "dim3")]
        self.gfx.set_color(color.x, color.y, color.z);
        self.color = color;
        self.base_color = color;
    }

    pub fn update(&mut self, _colliders: &DefaultColliderSet<f32>) {
        #[cfg(feature = "dim3")]
            node::update_scene_node(
                &mut self.gfx,
                _colliders,
                self.collider,
                &self.color,
                &self.delta,
            );
    }

    #[cfg(feature = "dim3")]
    pub fn scene_node(&self) -> &GraphicsNode {
        &self.gfx
    }

    #[cfg(feature = "dim3")]
    pub fn scene_node_mut(&mut self) -> &mut GraphicsNode {
        &mut self.gfx
    }

    pub fn object(&self) -> DefaultColliderHandle {
        self.collider
    }

    #[cfg(feature = "dim2")]
    pub fn draw(&mut self, window: &mut Window) {
        for vtx in self.vertices.chunks(2) {
            window.draw_planar_line(&vtx[0], &vtx[1], &self.color)
        }
    }
}
