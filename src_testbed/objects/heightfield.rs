#[cfg(feature = "dim3")]
use crate::objects::node::{self, GraphicsNode};
use kiss3d::window::Window;
#[cfg(feature = "dim3")]
use na::Point2;
use na::{self, Point3, RealField};
#[cfg(feature = "dim3")]
use ncollide::procedural::TriMesh;
use ncollide::shape;
#[cfg(feature = "dim3")]
use ncollide::transformation::ToTriMesh;
use nphysics::math::Isometry;
#[cfg(feature = "dim2")]
use nphysics::math::Point;
#[cfg(feature = "dim3")]
use nphysics::math::{Point, Vector};
use nphysics::object::{DefaultColliderHandle, DefaultColliderSet};

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
    pub fn new<N: RealField>(
        collider: DefaultColliderHandle,
        colliders: &DefaultColliderSet<N>,
        _: Isometry<f32>,
        heightfield: &shape::HeightField<N>,
        color: Point3<f32>,
        _: &mut Window,
    ) -> HeightField {
        let mut vertices = Vec::new();

        for seg in heightfield.segments() {
            vertices.push(na::convert::<Point<f64>, Point<f32>>(
                na::convert_unchecked(*seg.a()),
            ));
            vertices.push(na::convert::<Point<f64>, Point<f32>>(
                na::convert_unchecked(*seg.b()),
            ));
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
    pub fn new<N: RealField>(
        collider: DefaultColliderHandle,
        colliders: &DefaultColliderSet<N>,
        delta: Isometry<f32>,
        heightfield: &shape::HeightField<N>,
        color: Point3<f32>,
        window: &mut Window,
    ) -> HeightField {
        let mesh = heightfield.to_trimesh(());

        let mesh = TriMesh {
            coords: mesh
                .coords
                .into_iter()
                .map(|v| na::convert::<Point<f64>, Point<f32>>(na::convert_unchecked(v)))
                .collect(),
            normals: mesh.normals.map(|v| {
                v.into_iter()
                    .map(|v| na::convert::<Vector<f64>, Vector<f32>>(na::convert_unchecked(v)))
                    .collect()
            }),
            uvs: mesh.uvs.map(|v| {
                v.into_iter()
                    .map(|v| na::convert::<Point2<f64>, Point2<f32>>(na::convert_unchecked(v)))
                    .collect()
            }),
            indices: mesh.indices,
        };

        let mut res = HeightField {
            color,
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
        #[cfg(feature = "dim3")]
        self.gfx.set_color(color.x, color.y, color.z);
        self.color = color;
        self.base_color = color;
    }

    pub fn update<N: RealField>(&mut self, _colliders: &DefaultColliderSet<N>) {
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
