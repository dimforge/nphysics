extern crate nalgebra as na;
extern crate ncollide;
extern crate nphysics2d;
extern crate nphysics_testbed2d;
extern crate rand;

use na::{Vector1, Vector2, Translation2};
use ncollide::shape::{Plane, Cuboid};
use nphysics2d::world::World;
use nphysics2d::object::RigidBody;
use nphysics_testbed2d::Testbed;
use rand::distributions::{IndependentSample, Range, Weighted, WeightedChoice};
use nphysics_testbed2d::{CallBackMode, CallBackId};

fn main() {
    /*
     * World
     */
    let mut world = World::new();
    world.set_gravity(Vector2::new(0.0, 9.81));

    /*
     * First plane
     */
    let mut rb = RigidBody::new_static(Plane::new(Vector2::new(-1.0, -1.0)), 0.3, 0.6);

    rb.append_translation(&Translation2::new(0.0, 10.0));

    world.add_rigid_body(rb);

    /*
     * Second plane
     */
    let mut rb = RigidBody::new_static(Plane::new(Vector2::new(1.0, -1.0)), 0.3, 0.6);

    rb.append_translation(&Translation2::new(0.0, 10.0));

    world.add_rigid_body(rb);
    
    /*
     * Create the convex shapes
     */
    let num = (1000.0f32.sqrt()) as usize;
    let rad = 0.5;
    let shift   = 2.5 * rad;
    let centerx = shift * (num as f32) / 2.0;
    let centery = shift * (num as f32) / 2.0;
    //let g = 9.81;

    let mut handles = Vec::new();
    let mut forces = Vec::new();
    let mut ang_forces = Vec::new();
    let mut impulses = Vec::new();
    let mut torques = Vec::new();
    let mut rng = rand::thread_rng();
    let posneg = Range::new(-1f32, 1f32);
    let pos = Range::new(0.1f32, 1f32);
    let mut items = vec!(Weighted { weight: 1, item: -1.0 },
                         Weighted { weight: 1, item:  1.0 });
    let wc = WeightedChoice::new(&mut items);

    for i in 0usize .. num {
        for j in 0usize .. num {
            let x = i as f32 * shift - centerx;
            let y = j as f32 * shift - centery * 2.0 - 10.0;

            let geom = Cuboid::new(Vector2::new(0.5, 0.5));
            let mut rb = RigidBody::new_dynamic(geom, 0.1, 0.3, 0.6);
            rb.append_translation(&Translation2::new(x, y));
            let rb_handle = world.add_rigid_body(rb);
            
            handles.push(rb_handle);
            forces.push(Vector2::new(posneg.ind_sample(&mut rng) * 0.00008, pos.ind_sample(&mut rng) * -0.0008));
            ang_forces.push(Vector1::new(pos.ind_sample(&mut rng) * 0.001 * wc.ind_sample(&mut rng)));
            impulses.push(Vector2::new(posneg.ind_sample(&mut rng) * 0.1, pos.ind_sample(&mut rng) * -1.0));
            torques.push(Vector1::new(posneg.ind_sample(&mut rng) * 0.1));
        }
    }

    println!("Press 1 to toggle linear upward forces buildup.");
    println!("Press 2 to toggle angular forces buildup.");
    println!("Press 3 to apply an upward central impulse to the pieces.");
    println!("Press 4 to apply a torque impulse to the pieces.");

    /*
     * Run the simulation.
     */
    let mut testbed = Testbed::new(world);

    let ang_handles = handles.clone();
    let imp_handles = handles.clone();
    let torque_handles = handles.clone();

    testbed.add_callback(CallBackId::Cb1, Box::new(
            move | mode: CallBackMode |
            match mode {
                CallBackMode::StateActivated => {
                    println!("Linear Forces accumulation activated.");
                },
                CallBackMode::StateDeactivated => {
                    handles.iter().map(|object| object.borrow_mut().clear_linear_force()).last();
                    println!("Linear Forces deactivated.");
                },
                CallBackMode::LoopActive => {
                    handles.iter().zip(forces.iter()).map(
                        |(object, force)| {
                            let mut obj = object.borrow_mut();
                            let thr = obj.deactivation_threshold().unwrap_or(0.0);
                            obj.activate(thr*4.0);
                            obj.append_lin_force(force.clone());
                        }
                        ).last();
                },
                _ => {}
            }));

    testbed.add_callback(CallBackId::Cb2, Box::new(
            move | mode: CallBackMode |
            match mode {
                CallBackMode::StateActivated => {
                    println!("Angular Forces accumulation activated.");
                },
                CallBackMode::StateDeactivated => {
                    ang_handles.iter().map(|object| object.borrow_mut().clear_angular_force()).last();
                    println!("Angular Forces deactivated.");
                },
                CallBackMode::LoopActive => {
                    ang_handles.iter().zip(ang_forces.iter()).map(
                        |(object, force)| {
                            let mut obj = object.borrow_mut();
                            let thr = obj.deactivation_threshold().unwrap_or(0.0);
                            obj.activate(thr * 4.0);
                            obj.append_ang_force(force.clone());
                        }
                        ).last();
                },
                _ => {}
            }));

    testbed.add_callback(CallBackId::Cb3, Box::new(
            move | mode: CallBackMode |
            match mode {
                CallBackMode::StateActivated | CallBackMode::StateDeactivated => {
                    imp_handles.iter().zip(impulses.iter()).map(
                        |(object, impulse)| {
                            let mut obj = object.borrow_mut();
                            obj.apply_central_impulse(impulse.clone());
                        }
                        ).last();
                    println!("Nudge applied.");
                },
                _ => {}
            }));

    testbed.add_callback(CallBackId::Cb4, Box::new(
            move | mode: CallBackMode |
            match mode {
                CallBackMode::StateActivated | CallBackMode::StateDeactivated => {
                    torque_handles.iter().zip(torques.iter()).map(
                        |(object, torque)| {
                            let mut obj = object.borrow_mut();
                            obj.apply_angular_momentum(torque.clone());
                        }
                        ).last();
                    println!("Angular momentum applied.");
                },
                _ => {}
            }));

    testbed.run();
}
