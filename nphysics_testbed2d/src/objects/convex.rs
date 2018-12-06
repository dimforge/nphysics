use kiss3d::scene::PlanarSceneNode;
use kiss3d::window::Window;
use na::{Isometry2, Point2, Point3, Vector2};
use nphysics2d::object::ColliderHandle;
use nphysics2d::world::World;
use crate::objects::node;

pub struct Convex {
    color: Point3<f32>,
    base_color: Point3<f32>,
    delta: Isometry2<f32>,
    gfx: PlanarSceneNode,
    collider: ColliderHandle,
}

impl Convex {
    pub fn new(
        collider: ColliderHandle,
        world: &World<f32>,
        delta: Isometry2<f32>,
        vertices: Vec<Point2<f32>>,
        color: Point3<f32>,
        window: &mut Window,
    ) -> Convex {
        let mut res = Convex {
            color: color,
            base_color: color,
            delta: delta,
            gfx: window.add_convex_polygon(vertices, Vector2::from_element(1.0)),
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
