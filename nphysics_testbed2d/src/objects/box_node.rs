use kiss3d::scene::PlanarSceneNode;
use kiss3d::window;
use na::{Isometry2, Point3};
use nphysics2d::object::ColliderHandle;
use nphysics2d::world::World;
use objects::node;

pub struct Box {
    color: Point3<f32>,
    base_color: Point3<f32>,
    delta: Isometry2<f32>,
    gfx: PlanarSceneNode,
    collider: ColliderHandle,
}

impl Box {
    pub fn new(
        collider: ColliderHandle,
        world: &World<f32>,
        delta: Isometry2<f32>,
        rx: f32,
        ry: f32,
        color: Point3<f32>,
        window: &mut window::Window,
    ) -> Box {
        let gx = rx * 2.0;
        let gy = ry * 2.0;

        let mut res = Box {
            color,
            base_color: color,
            delta,
            gfx: window.add_rectangle(gx, gy),
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

    pub fn scene_node(&self) -> &PlanarSceneNode {
        &self.gfx
    }

    pub fn scene_node_mut(&mut self) -> &mut PlanarSceneNode {
        &mut self.gfx
    }

    pub fn object(&self) -> ColliderHandle {
        self.collider
    }
}
