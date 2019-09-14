use crate::objects::node::{self, GraphicsNode};
use kiss3d::window::Window;
use na::Point3;
#[cfg(feature = "dim3")]
use ncollide::procedural::TriMesh;
#[cfg(feature = "dim2")]
use nphysics::math::Point;
use nphysics::math::{Isometry, Vector};
use nphysics::object::{DefaultColliderHandle, DefaultColliderSet};

pub struct Convex {
    color: Point3<f32>,
    base_color: Point3<f32>,
    delta: Isometry<f32>,
    gfx: GraphicsNode,
    collider: DefaultColliderHandle,
}

impl Convex {
    #[cfg(feature = "dim2")]
    pub fn new(
        collider: DefaultColliderHandle,
        colliders: &DefaultColliderSet<f32>,
        delta: Isometry<f32>,
        vertices: Vec<Point<f32>>,
        color: Point3<f32>,
        window: &mut Window,
    ) -> Convex
    {
        let mut res = Convex {
            color,
            base_color: color,
            delta,
            gfx: window.add_convex_polygon(vertices, Vector::from_element(1.0)),
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

        res.gfx.set_color(color.x, color.y, color.z);
        res.gfx
            .set_local_transformation(colliders.get(collider).unwrap().position() * res.delta);
        res.update(colliders);

        res
    }

    #[cfg(feature = "dim3")]
    pub fn new(
        collider: DefaultColliderHandle,
        colliders: &DefaultColliderSet<f32>,
        delta: Isometry<f32>,
        convex: &TriMesh<f32>,
        color: Point3<f32>,
        window: &mut Window,
    ) -> Convex
    {
        let mut res = Convex {
            color,
            base_color: color,
            delta,
            gfx: window.add_trimesh(convex.clone(), Vector::from_element(1.0)),
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
        self.gfx.set_color(color.x, color.y, color.z);
        self.color = color;
        self.base_color = color;
    }

    pub fn update(&mut self, colliders: &DefaultColliderSet<f32>) {
        node::update_scene_node(
            &mut self.gfx,
            colliders,
            self.collider,
            &self.color,
            &self.delta,
        );
    }

    pub fn scene_node(&self) -> &GraphicsNode {
        &self.gfx
    }

    pub fn scene_node_mut(&mut self) -> &mut GraphicsNode {
        &mut self.gfx
    }

    pub fn object(&self) -> DefaultColliderHandle {
        self.collider
    }
}
