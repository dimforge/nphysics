use std::ptr;
use nalgebra::traits::scalar_op::ScalarMul;
use ncollide::util::hash_map::HashMap;
use ncollide::util::hash::UintTWHash;
use object::body::{Body, RigidBody, SoftBody};
use integration::integrator::Integrator;

pub struct BodyDamping<N, LV, AV, M, II> {
    priv damping_factor: N,
    priv objects:        HashMap<uint, @mut Body<N, LV, AV, M, II>, UintTWHash>
}

impl<N, LV, AV, M, II> BodyDamping<N, LV, AV, M, II> {
    #[inline]
    pub fn new(damping_factor: N) -> BodyDamping<N, LV, AV, M, II> {
        BodyDamping {
            damping_factor: damping_factor,
            objects:        HashMap::new(UintTWHash)
        }
    }
}

impl<N: Clone, LV: Clone + ScalarMul<N>, AV: Clone + ScalarMul<N>, M: Clone, II: Clone>
Integrator<N, Body<N, LV, AV, M, II>> for BodyDamping<N, LV, AV, M, II> {
    #[inline]
    fn add(&mut self, o: @mut Body<N, LV, AV, M, II>) {
        self.objects.insert(ptr::to_mut_unsafe_ptr(o) as uint, o);
    }

    #[inline]
    fn remove(&mut self, o: @mut Body<N, LV, AV, M, II>) {
        self.objects.remove(&(ptr::to_mut_unsafe_ptr(o) as uint));
    }

    #[inline]
    fn activate(&mut self, o: @mut Body<N, LV, AV, M, II>) {
        self.add(o);
    }

    #[inline]
    fn deactivate(&mut self, o: @mut Body<N, LV, AV, M, II>) {
        self.remove(o)
    }

    fn pre_update(&mut self, _: N) {
        for o in self.objects.elements().iter() {
            match *o.value {
                RigidBody(rb) => {
                    let new_lin = rb.lin_vel().scalar_mul(&self.damping_factor);
                    rb.set_lin_vel(new_lin);
                    let new_ang = rb.ang_vel().scalar_mul(&self.damping_factor);
                    rb.set_ang_vel(new_ang);
                },
                SoftBody(_) => {
                    fail!("Not yet implemented.")
                }
            }
        }
    }

    #[inline]
    fn post_update(&mut self, _: N) {
    }
}
