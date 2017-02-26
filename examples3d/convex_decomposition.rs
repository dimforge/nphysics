extern crate rand;
extern crate kiss3d;
extern crate nalgebra as na;
extern crate ncollide;
extern crate nphysics3d;
extern crate nphysics_testbed3d;

use std::path::Path;
use rand::random;
use na::{Point3, Vector3, Translation3};
use kiss3d::loader::obj;
use ncollide::shape::{Plane, Compound, ConvexHull, ShapeHandle};
use ncollide::procedural::TriMesh3;
use ncollide::transformation;
use ncollide::bounding_volume::{BoundingVolume, AABB};
use ncollide::bounding_volume;
use nphysics3d::world::World;
use nphysics3d::object::RigidBody;
use nphysics_testbed3d::Testbed;

fn main() {
    /*
     * World
     */
    let mut world = World::new();
    world.set_gravity(Vector3::new(0.0, -9.81, 0.0));

    /*
     * Planes
     */
    let shift = 10.0;

    let normals = [
        Vector3::new(0.0, 1.0, 0.0),
        Vector3::new(-1.0, 1.0, 0.0),
        Vector3::new(1.0, 1.0, 0.0),
        Vector3::new(0.0, 1.0, -1.0),
        Vector3::new(0.0, 1.0, 1.0),
    ];
    let poss = [
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(shift, 0.0, 0.0),
        Vector3::new(-shift, 0.0, 0.0),
        Vector3::new(0.0, 0.0, shift),
        Vector3::new(0.0, 0.0, -shift)
    ];

    for (normal, pos) in normals.iter().zip(poss.iter()) {
        let geom = Plane::new(*normal);
        let mut rb = RigidBody::new_static(geom, 0.3, 0.6);

        rb.append_translation(&Translation3::from_vector(*pos));

        world.add_rigid_body(rb);
    }

    /*
     * Create the convex decompositions.
     */

    let geoms = models();
    let mut bodies = Vec::new();
    let ngeoms = geoms.len();

    for obj_path in geoms.into_iter() {
        let deltas   = na::one();
        let mtl_path = Path::new("");


        let mut geom_data = Vec::new();

        let obj = obj::parse_file(&Path::new(&obj_path), &mtl_path, "");

        if let Ok(model) = obj {
            let meshes: Vec<TriMesh3<f32>> = model.into_iter().map(|mesh| mesh.1.to_trimesh().unwrap()).collect();

            // Compute the size of the model, to scale it and have similar size for everything.
            let (mins, maxs) = bounding_volume::point_cloud_aabb(&deltas, &meshes[0].coords[..]);
            let mut aabb = AABB::new(mins, maxs);

            for mesh in meshes[1 ..].iter() {
                let (mins, maxs) = bounding_volume::point_cloud_aabb(&deltas, &mesh.coords[..]);
                aabb.merge(&AABB::new(mins, maxs));
            }

            let center = aabb.center().coords;
            let diag = na::norm(&(*aabb.maxs() - *aabb.mins()));

            for mut trimesh in meshes.into_iter() {
                trimesh.translate_by(&Translation3::from_vector(-center));
                trimesh.scale_by_scalar(6.0 / diag);
                trimesh.split_index_buffer(true);

                let (decomp, _) = transformation::hacd(trimesh, 0.03, 1);

                for mesh in decomp.into_iter() {
                    let convex = ShapeHandle::new(ConvexHull::new(mesh.coords));
                    geom_data.push((deltas, convex));
                }
            }

            let compound = Compound::new(geom_data);

            let mut rb = RigidBody::new_dynamic(compound, 1.0, 0.3, 0.5);
            rb.set_deactivation_threshold(Some(0.5));

            bodies.push(rb)
        }
    }

    if bodies.len() != ngeoms {
        println!("#########################################################################################");
        println!("Some model are missing. You can download them all at : http://crozet.re/nphysics/models.");
        println!("All the obj files should be put on the `./media/models` folder.");
        println!("#########################################################################################");
    }

    let nreplicats = 100 / bodies.len();

    for rb in bodies.iter() {
        for _ in 0 .. nreplicats {
            let mut rb = rb.clone();
            let pos = random::<Vector3<f32>>() * 30.0 + Vector3::new(-15.0, 15.0, -15.0);
            rb.append_translation(&Translation3::from_vector(pos));

            world.add_rigid_body(rb);
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
    vec!("media/models/CRATERS_F_decimated.obj".to_string()
         , "media/models/DRAGON_F_decimated.obj".to_string()
         , "media/models/GARGOYLE_F_decimated.obj".to_string()
         , "media/models/Hand1_decimated.obj".to_string()
         , "media/models/RSCREATURE_F_decimated.obj".to_string()
         , "media/models/Sketched-Brunnen_decimated.obj".to_string()
         , "media/models/Teapot_decimated.obj".to_string()
         , "media/models/block.obj".to_string()
         , "media/models/block_decimated.obj".to_string()
         , "media/models/bowl.obj".to_string()
         , "media/models/bunny_decimated.obj".to_string()
         , "media/models/camel.obj".to_string()
         , "media/models/camel_decimated.obj".to_string()
         , "media/models/casting.obj".to_string()
         , "media/models/casting_decimated.obj".to_string()
         , "media/models/chair.obj".to_string()
         , "media/models/cow1.obj".to_string()
         , "media/models/cow1_decimated.obj".to_string()
         , "media/models/cow2.obj".to_string()
         , "media/models/cow2_decimated.obj".to_string()
         , "media/models/crank_decimated.obj".to_string()
         , "media/models/cup.obj".to_string()
         , "media/models/cup_decimated.obj".to_string()
         , "media/models/dancer2_decimated.obj".to_string()
         , "media/models/deer_bound.obj".to_string()
         , "media/models/dilo_decimated.obj".to_string()
         , "media/models/dino_decimated.obj".to_string()
         , "media/models/drum.obj".to_string()
         , "media/models/egea.obj".to_string()
         , "media/models/egea_decimated.obj".to_string()
         , "media/models/eight.obj".to_string()
         , "media/models/elephant_decimated.obj".to_string()
         , "media/models/elk.obj".to_string()
         , "media/models/elk_decimated.obj".to_string()
         , "media/models/face-YH_decimated.obj".to_string()
         , "media/models/feline_decimated.obj".to_string()
         , "media/models/fish_decimated.obj".to_string()
         , "media/models/foot.obj".to_string()
         , "media/models/foot_decimated.obj".to_string()
         , "media/models/genus3.obj".to_string()
         , "media/models/genus3_decimated.obj".to_string()
         , "media/models/greek_sculpture_decimated.obj".to_string()
         , "media/models/hand2_decimated.obj".to_string()
         , "media/models/hand_decimated.obj".to_string()
         , "media/models/helix.obj".to_string()
         , "media/models/helmet.obj".to_string()
         , "media/models/hero.obj".to_string()
         , "media/models/hero_decimated.obj".to_string()
         , "media/models/homer.obj".to_string()
         , "media/models/homer_decimated.obj".to_string()
         , "media/models/hornbug.obj".to_string()
         , "media/models/horse_decimated.obj".to_string()
         , "media/models/maneki-neko_decimated.obj".to_string()
         , "media/models/mannequin-devil.obj".to_string()
         , "media/models/mannequin-devil_decimated.obj".to_string()
         , "media/models/mannequin.obj".to_string()
         , "media/models/mannequin_decimated.obj".to_string()
         , "media/models/mask_decimated.obj".to_string()
         , "media/models/moaimoai.obj".to_string()
         , "media/models/moaimoai_decimated.obj".to_string()
         , "media/models/monk_decimated.obj".to_string()
         , "media/models/octopus_decimated.obj".to_string()
         , "media/models/pig.obj".to_string()
         , "media/models/pig_decimated.obj".to_string()
         , "media/models/pinocchio_b_decimated.obj".to_string()
         , "media/models/polygirl.obj".to_string()
         , "media/models/polygirl_decimated.obj".to_string()
         , "media/models/rabbit_decimated.obj".to_string()
         , "media/models/rocker-arm.obj".to_string()
         , "media/models/rocker-arm_decimated.obj".to_string()
         , "media/models/screw-remeshed_decimated.obj".to_string()
         , "media/models/screwdriver_decimated.obj".to_string()
         , "media/models/shark_b_decimated.obj".to_string()
         , "media/models/skull-original_decimated.obj".to_string()
         , "media/models/sledge.obj".to_string()
         , "media/models/squirrel.obj".to_string()
         , "media/models/squirrel_decimated.obj".to_string()
         , "media/models/sword_decimated.obj".to_string()
         , "media/models/table.obj".to_string()
         , "media/models/test2.obj".to_string()
         , "media/models/tstTorusModel.obj".to_string()
         , "media/models/tstTorusModel2.obj".to_string()
         , "media/models/tstTorusModel3.obj".to_string()
         , "media/models/tube1.obj".to_string()
         , "media/models/venus-original_decimated.obj".to_string()
         , "media/models/venus.obj".to_string()
        )
}
