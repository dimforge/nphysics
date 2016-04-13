extern crate nalgebra as na;
extern crate ncollide;
extern crate nphysics3d;
extern crate nphysics_testbed3d;

use na::{Pnt3, Vec3, Iso3, Translation};
use ncollide::shape::{Plane, Cuboid, Compound, ShapeHandle};
use nphysics3d::volumetric::Volumetric;
use nphysics3d::world::World;
use nphysics3d::object::RigidBody;
use nphysics_testbed3d::Testbed;

fn main() {
    /*
     * World
     */
    let mut world = World::new();
    world.set_gravity(Vec3::new(0.0, -9.81, 0.0));

    /*
     * Planes
     */
    let rb = RigidBody::new_static(Plane::new(Vec3::new(0.0, 1.0, 0.0)), 0.3, 0.6);

    world.add_rigid_body(rb);

    /*
     * Cross shaped geometry
     */
    let delta1 = Iso3::new(Vec3::new(0.0, -5.0, 0.0), na::zero());
    let delta2 = Iso3::new(Vec3::new(-5.0, 0.0, 0.0), na::zero());
    let delta3 = Iso3::new(Vec3::new(5.0, 0.0, 0.0), na::zero());

    let mut cross_geoms = Vec::new();
    let vertical   = ShapeHandle::new(Cuboid::new(Vec3::new(0.21f32, 4.96, 0.21)));
    let horizontal = ShapeHandle::new(Cuboid::new(Vec3::new(4.96f32, 0.21, 0.21)));
    cross_geoms.push((delta1, horizontal));
    cross_geoms.push((delta2, vertical.clone()));
    cross_geoms.push((delta3, vertical));

    let compound = Compound::new(cross_geoms);
    let mass     = compound.mass_properties(1.0);
    let cross    = ShapeHandle::new(compound);

    /*
     * Create the crosses
     */
    let num     = 6;
    let rad     = 5.0;
    let shift   = rad * 2.0;
    let centerx = shift * (num as f32) / 2.0;
    let centery = 30.0 + shift / 2.0;
    let centerz = shift * (num as f32) / 2.0;

    for i in 0usize .. num {
        for j in 0usize .. num {
            for k in 0usize .. num {
                let x = i as f32 * shift - centerx;
                let y = j as f32 * shift + centery;
                let z = k as f32 * shift - centerz;

                let mut rb = RigidBody::new(cross.clone(), Some(mass), 0.3, 0.5);

                rb.append_translation(&Vec3::new(x, y, z));

                world.add_rigid_body(rb);
            }
        }
    }

    /*
     * Set up the testbed.
     */
    let mut testbed = Testbed::new(world);

    testbed.look_at(Pnt3::new(-30.0, 30.0, -30.0), Pnt3::new(0.0, 0.0, 0.0));
    testbed.run();
}
