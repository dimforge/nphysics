use kiss3d::scene::SceneNode;
use kiss3d::window;
use na::{Point3, Vector3};
use nphysics3d::object::ColliderHandle;
use nphysics3d::world::World;
use num::Zero;

pub struct Plane {
    gfx: SceneNode,
    collider: ColliderHandle,
}

impl Plane {
    pub fn new(
        collider: ColliderHandle,
        world: &World<f32>,
        world_pos: &Point3<f32>,
        world_normal: &Vector3<f32>,
        color: Point3<f32>,
        window: &mut window::Window,
    ) -> Plane {
        let mut res = Plane {
            gfx: window.add_quad(100.0, 100.0, 10, 10),
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

        let up = if world_normal.z.is_zero() && world_normal.y.is_zero() {
            Vector3::z()
        } else {
            Vector3::x()
        };

        res.gfx
            .reorient(world_pos, &(*world_pos + *world_normal), &up);

        res.update(world);

        res
    }

    pub fn select(&mut self) {}

    pub fn unselect(&mut self) {}

    pub fn update(&mut self, _: &World<f32>) {
        // FIXME: atm we assume the plane does not move
    }

    pub fn set_color(&mut self, color: Point3<f32>) {
        self.gfx.set_color(color.x, color.y, color.z);
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
