extern crate kiss3d;
extern crate nalgebra as na;
extern crate ncollide3d;
extern crate nphysics3d;
extern crate nphysics_testbed3d;
extern crate rand;

use std::path::Path;
use na::{Isometry3, Point3, Translation3, Vector3};
use kiss3d::loader::obj;
use ncollide3d::shape::{Compound, ConvexHull, Cuboid, ShapeHandle};
use ncollide3d::procedural::TriMesh;
use ncollide3d::transformation;
use ncollide3d::bounding_volume::{self, BoundingVolume, AABB};
use nphysics3d::world::World;
use nphysics3d::object::{ColliderDesc, RigidBodyDesc};
use nphysics_testbed3d::Testbed;


fn main() {
    /*
     * World
     */
    let mut world = World::new();
    world.set_gravity(Vector3::new(0.0, -9.81, 0.0));

    /*
     * Ground
     */
    let ground_size = 50.0;
    let ground_shape =
        ShapeHandle::new(Cuboid::new(Vector3::repeat(ground_size)));

    ColliderDesc::new(ground_shape)
        .with_translation(Vector3::y() * -ground_size)
        .build(&mut world);

    /*
     * Create the convex decompositions.
     */
    let geoms = models();
    let ngeoms = geoms.len();
    let width = (ngeoms as f32).sqrt() as usize;
    let num_duplications = 4;
    let shift = 5.0f32;

    for (igeom, obj_path) in geoms.into_iter().enumerate() {
        let deltas = na::one();
        let mtl_path = Path::new("");

        let mut geom_data = Vec::new();

        let obj = obj::parse_file(&Path::new(&obj_path), &mtl_path, "");

        if let Ok(model) = obj {
            let meshes: Vec<TriMesh<f32>> = model
                .into_iter()
                .map(|mesh| mesh.1.to_trimesh().unwrap())
                .collect();

            // Compute the size of the model, to scale it and have similar size for everything.
            let mut aabb = bounding_volume::point_cloud_aabb(&deltas, &meshes[0].coords[..]);

            for mesh in meshes[1..].iter() {
                aabb.merge(&bounding_volume::point_cloud_aabb(&deltas, &mesh.coords[..]));
            }

            let center = aabb.center().coords;
            let diag = na::norm(&(*aabb.maxs() - *aabb.mins()));

            for mut trimesh in meshes.into_iter() {
                trimesh.translate_by(&Translation3::from_vector(-center));
                trimesh.scale_by_scalar(6.0 / diag);
                trimesh.split_index_buffer(true);

                let (decomp, _) = transformation::hacd(trimesh, 0.03, 1);

                for hull in decomp.into_iter() {
                    let indices: Vec<usize> = hull.flat_indices()
                        .into_iter()
                        .map(|i| i as usize)
                        .collect();
                    let vertices = hull.coords;

                    if let Some(chull) = ConvexHull::try_new(vertices, &indices) {
                        let convex = ShapeHandle::new(chull);
                        geom_data.push((deltas, convex));
                    }
                }
            }

            let compound = Compound::new(geom_data);
            let geom = ShapeHandle::new(compound);
            let collider_desc = ColliderDesc::new(geom)
                .with_density(1.0);
            let mut rb_desc = RigidBodyDesc::default()
                .with_collider(&collider_desc);

            for k in 1..num_duplications + 1 {
                let i = igeom % width;
                let j = igeom / width;
                let pos = Vector3::new(i as f32, k as f32, j as f32) * shift;

                rb_desc
                    .set_translation(pos)
                    .build(&mut world);
            }
        }
    }

    /*
     * Set up the testbed.
     */
    let mut testbed = Testbed::new(world);

    testbed.look_at(Point3::new(-30.0, 30.0, -30.0), Point3::new(0.0, 0.0, 0.0));
    testbed.run();
}

fn models() -> Vec<String> {
    vec![
        "media/models/camel_decimated.obj".to_string(),
        "media/models/chair.obj".to_string(),
        "media/models/cup_decimated.obj".to_string(),
        // "media/models/dilo_decimated.obj".to_string(),
        "media/models/feline_decimated.obj".to_string(),
        "media/models/genus3_decimated.obj".to_string(),
        "media/models/hand2_decimated.obj".to_string(),
        "media/models/hand_decimated.obj".to_string(),
        "media/models/hornbug.obj".to_string(),
        // "media/models/octopus_decimated.obj".to_string(),
        "media/models/rabbit_decimated.obj".to_string(),
        "media/models/screwdriver_decimated.obj".to_string(),
        "media/models/table.obj".to_string(),
        "media/models/tstTorusModel.obj".to_string(),
        "media/models/tstTorusModel2.obj".to_string(),
        "media/models/tstTorusModel3.obj".to_string(),
        "media/models/tube1.obj".to_string(),
    ]
}
