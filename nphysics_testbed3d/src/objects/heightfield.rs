use kiss3d::resource;
use kiss3d::scene::SceneNode;
use kiss3d::window::Window;
use na::{self, Isometry3, Point3, Vector3, DMatrix};
use nphysics3d::object::{ColliderHandle, ColliderAnchor};
use nphysics3d::world::World;
use crate::objects::node;
use std::cell::RefCell;
use std::rc::Rc;

pub struct HeightField {
    color: Point3<f32>,
    base_color: Point3<f32>,
    delta: Isometry3<f32>,
    gfx: SceneNode,
    collider: ColliderHandle,
}

impl HeightField {
    pub fn new(
        collider: ColliderHandle,
        world: &World<f32>,
        delta: Isometry3<f32>,
        heights: &DMatrix<f32>,
        scale: &Vector3<f32>,
        color: Point3<f32>,
        window: &mut Window,
    ) -> HeightField {
        let mut quad = ncollide3d::procedural::quad(1.0, 1.0, heights.nrows() - 1, heights.ncols() - 1);

        // ncollide generates a quad with `z` as the normal.
        // so we switch z and y here and set the right altitude at each point.
        for (i, p) in quad.coords.iter_mut().enumerate() {
            p.z = p.y;
            p.y = heights[i];
        }

        quad.flip_triangles();
        quad.replicate_vertices();
        quad.recompute_normals();

        let mut res = HeightField {
            color: color,
            base_color: color,
            delta: delta,
            gfx: window.add_trimesh(quad, *scale),
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
