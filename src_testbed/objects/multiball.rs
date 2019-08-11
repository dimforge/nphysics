use kiss3d::window::Window;
use na::Point3;
use ncollide::shape;
use nphysics::math::{Isometry, Point};
use nphysics::object::{DefaultColliderHandle, DefaultColliderSet, ColliderAnchor};
use crate::objects::node::{self, GraphicsNode};

pub struct Multiball {
    color: Point3<f32>,
    base_color: Point3<f32>,
    delta: Isometry<f32>,
    gfx: GraphicsNode,
    balls_gfx: Vec<GraphicsNode>,
    collider: DefaultColliderHandle,
}

impl Multiball {
    pub fn new(
        collider: DefaultColliderHandle,
        colliders: &DefaultColliderSet<f32>,
        delta: Isometry<f32>,
        radius: f32,
        centers: &[Point<f32>],
        color: Point3<f32>,
        window: &mut Window,
    ) -> Multiball {
        #[cfg(feature = "dim2")]
            let mut gfx = window.add_planar_group();
        #[cfg(feature = "dim3")]
            let mut gfx = window.add_group();

        let mut balls_gfx = Vec::new();

        for c in centers {
            #[cfg(feature = "dim2")]
                let mut ball_gfx = gfx.add_circle(radius);
            #[cfg(feature = "dim3")]
                let mut ball_gfx = gfx.add_sphere(radius);
            ball_gfx.set_local_translation(c.coords.into());
            balls_gfx.push(ball_gfx);
        }

        let mut res = Multiball {
            color,
            base_color: color,
            delta,
            gfx,
            balls_gfx,
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

        // res.gfx.set_texture_from_file(&Path::new("media/kitten.png"), "kitten");
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

        // Update if some deformation occurred.
        // FIXME: don't update if it did not move.
        if let Some(c) = colliders.get(self.collider) {
            if let ColliderAnchor::OnDeformableBody { .. } = c.anchor() {
                let shape = c.shape();
                let shape = shape.as_shape::<shape::Multiball<f32>>().unwrap();
                let vtx = shape.centers();

                for (i, pt) in shape.centers().iter().enumerate() {
                    self.balls_gfx[i].set_local_translation(pt.coords.into())
                }
            }
        }
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
