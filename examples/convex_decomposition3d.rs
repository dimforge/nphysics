#![crate_type = "bin"]
#![warn(non_camel_case_types)]

extern crate native;
extern crate rustrt;
extern crate kiss3d;
extern crate graphics3d;
extern crate nphysics = "nphysics3df32";
extern crate ncollide = "ncollide3df32";
extern crate nalgebra;

use rustrt::bookkeeping;
use std::sync::{Arc, RWLock};
use std::rand;
use std::rc::Rc;
use std::cell::RefCell;
use nalgebra::na::{Vec3, Translation};
use nalgebra::na;
use kiss3d::window::Window;
use kiss3d::loader::obj;
use ncollide::geom::{Plane, Convex, Compound, CompoundData};
use ncollide::procedural::TriMesh;
use ncollide::procedural;
use ncollide::bounding_volume::{BoundingVolume, AABB};
use ncollide::bounding_volume;
use nphysics::world::World;
use nphysics::object::RigidBody;
use graphics3d::engine::GraphicsManager;

#[start]
fn start(argc: int, argv: *const *const u8) -> int {
    native::start(argc, argv, main)
}

fn main() {
    GraphicsManager::simulate(convex_3d)
}

pub fn convex_3d(window: &mut Window, graphics: &mut GraphicsManager) -> World {
    /*
     * World
     */
    let mut world = World::new();
    world.set_gravity(Vec3::new(0.0f32, -9.81, 0.0));

    /*
     * Planes
     */
    let geom = Plane::new(Vec3::new(0.0f32, 1.0, 0.0));
    let body = Rc::new(RefCell::new(RigidBody::new_static(geom, 0.3, 0.6)));

    world.add_body(body.clone());
    graphics.add(window, body);

    /*
     * Create the convex decompositions.
     */

    let geoms = models();
    let bodies = Arc::new(RWLock::new(Vec::new()));

    for obj_path in geoms.move_iter() {
        let deltas   = na::one();
        let mtl_path = Path::new("");

        let bodies = bodies.clone();

        spawn(proc() {
            let mut geom_data = CompoundData::new();

            let model  = obj::parse_file(&Path::new(obj_path), &mtl_path, "").unwrap();
            let meshes: Vec<TriMesh<f32, Vec3<f32>>> = model.move_iter().map(|mesh| mesh.ref1().to_trimesh().unwrap()).collect();

            // Compute the size of the model, to scale it and have similar size for everything.
            let (mins, maxs) = bounding_volume::point_cloud_aabb(&deltas, meshes[0].coords.as_slice());
            let mut aabb = AABB::new(mins, maxs);

            for mesh in meshes.slice_from(1).iter() {
                let (mins, maxs) = bounding_volume::point_cloud_aabb(&deltas, mesh.coords.as_slice());
                aabb.merge(&AABB::new(mins, maxs));
            }

            let center = aabb.translation();
            let diag = na::norm(&(*aabb.maxs() - *aabb.mins()));

            for mut trimesh in meshes.move_iter() {
                trimesh.translate_by(&-center);
                trimesh.scale_by_scalar(&(6.0 / diag));
                trimesh.split_index_buffer(true);

                let (decomp, _) = procedural::hacd(trimesh, 0.03, 1);

                for comp in decomp.move_iter() {
                    // FIXME: add to Convex a method to modify its margin.
                    let mesh = unsafe { Convex::new_with_convex_mesh(comp.unwrap(), 0.04) };

                    geom_data.push_geom(deltas, mesh, 1.0);
                }
            }

            let compound = Compound::new(geom_data);

            let mut rb = RigidBody::new_dynamic(compound, 1.0f32, 0.3, 0.5);
            rb.set_deactivation_threshold(None);

            // Position them randomly.
            let pos = rand::random::<Vec3<f32>>() * 50.0f32 + Vec3::new(0.0f32, 2.0, 0.0);
            rb.append_translation(&pos);


            bodies.write().push(rb);
        })
    }

    bookkeeping::wait_for_other_tasks();

    for rb in bodies.read().iter() {
        let body = Rc::new(RefCell::new(rb.clone()));
        world.add_body(body.clone());
        graphics.add(window, body);
    }

    /*
     * Set up the camera and that is it!
     */
    graphics.look_at(Vec3::new(-30.0, 30.0, -30.0), Vec3::new(0.0, 0.0, 0.0));

    world
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
