extern crate nalgebra as na;
extern crate ncollide;
extern crate nphysics3d;
extern crate nphysics_testbed3d;

use na::{Point3, Vector3, Translation};
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
    world.set_gravity(Vector3::new(0.0, -9.81, 0.0));

    /*
     * Planes
     */
    let rb = RigidBody::new_static(Plane::new(Vector3::new(0.0, 1.0, 0.0)), 0.3, 0.6);

    world.add_rigid_body(rb);

    /*
     * Cross shaped geometry
     */
    let mut cross_geoms = Vec::new();

    let edge_x = Cuboid::new(Vector3::new(4.96f32, 0.21, 0.21));
    let edge_y = Cuboid::new(Vector3::new(0.21f32, 4.96, 0.21));
    let edge_z = Cuboid::new(Vector3::new(0.21f32, 0.21, 4.96));

    cross_geoms.push((na::one(), ShapeHandle::new(edge_x)));
    cross_geoms.push((na::one(), ShapeHandle::new(edge_y)));
    cross_geoms.push((na::one(), ShapeHandle::new(edge_z)));

    let compound = Compound::new(cross_geoms);
    let mass     = compound.mass_properties(1.0);
    let cross    = ShapeHandle::new(compound);

    /*
     * Create the crosses
     */
    let num     = 6;
    let rad     = 5.0;
    let shift   = (rad + 0.08) * 2.0;
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

                rb.append_translation(&Vector3::new(x, y, z));

                world.add_rigid_body(rb);
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
