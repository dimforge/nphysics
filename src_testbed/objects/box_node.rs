use kiss3d::window;
use na::Point3;
use nphysics::math::{Isometry, Vector};
use nphysics::world::World;
use nphysics::object::ColliderSlabHandle;
use crate::objects::node::{self, GraphicsNode};

pub struct Box {
    color: Point3<f32>,
    base_color: Point3<f32>,
    delta: Isometry<f32>,
    gfx: GraphicsNode,
    collider: ColliderSlabHandle,
}

impl Box {
    pub fn new(
        collider: ColliderSlabHandle,
        world: &World<f32>,
        delta: Isometry<f32>,
        half_extents: Vector<f32>,
        color: Point3<f32>,
        window: &mut window::Window,
    ) -> Box {
        let extents = half_extents * 2.0;
        #[cfg(feature = "dim2")]
            let node = window.add_rectangle(extents.x, extents.y);
        #[cfg(feature = "dim3")]
            let node = window.add_cube(extents.x, extents.y, extents.z);

        let mut res = Box {
            color,
            base_color: color,
            delta,
            gfx: node,
            collider,
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

    pub fn scene_node(&self) -> &GraphicsNode {
        &self.gfx
    }

    pub fn scene_node_mut(&mut self) -> &mut GraphicsNode {
        &mut self.gfx
    }

    pub fn object(&self) -> ColliderSlabHandle {
        self.collider
    }
}
