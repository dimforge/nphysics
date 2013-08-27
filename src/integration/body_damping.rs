use std::ptr;
use nalgebra::traits::vector::Vec;
use ncollide::util::hash_map::HashMap;
use ncollide::util::hash::UintTWHash;
use object::body::{Body, RigidBody, SoftBody};
use integration::integrator::Integrator;
use signal::signal::SignalEmiter;

pub struct BodyDamping<N, LV, AV, M, II> {
    priv damping_factor: N,
    priv objects:        HashMap<uint, @mut Body<N, LV, AV, M, II>, UintTWHash>
}

impl<N:  'static + Clone,
     LV: 'static + Clone + Vec<N>,
     AV: 'static + Clone + Vec<N>,
     M:  'static + Clone,
     II: 'static + Clone>
BodyDamping<N, LV, AV, M, II> {
    #[inline]
    pub fn new<C>(events:         &mut SignalEmiter<N, Body<N, LV, AV, M, II>, C>,
                  damping_factor: N)
                  -> @mut BodyDamping<N, LV, AV, M, II> {
        let res = @mut BodyDamping {
            damping_factor: damping_factor,
            objects:        HashMap::new(UintTWHash)
        };

        events.add_body_activated_handler(ptr::to_mut_unsafe_ptr(res) as uint, |o, _| res.add(o));
        events.add_body_deactivated_handler(ptr::to_mut_unsafe_ptr(res) as uint, |o| res.remove(o));

        res
    }
}

impl<N: Clone, LV: Clone + Vec<N>, AV: Clone + Vec<N>, M: Clone, II: Clone>
Integrator<N, Body<N, LV, AV, M, II>> for BodyDamping<N, LV, AV, M, II> {
    #[inline]
    fn add(&mut self, o: @mut Body<N, LV, AV, M, II>) {
        self.objects.insert(ptr::to_mut_unsafe_ptr(o) as uint, o);
    }

    #[inline]
    fn remove(&mut self, o: @mut Body<N, LV, AV, M, II>) {
        self.objects.remove(&(ptr::to_mut_unsafe_ptr(o) as uint));
    }

    fn update(&mut self, _: N) {
        for o in self.objects.elements().iter() {
            match *o.value {
                RigidBody(rb) => {
                    let new_lin = rb.lin_vel() * self.damping_factor;
                    rb.set_lin_vel(new_lin);
                    let new_ang = rb.ang_vel() * self.damping_factor;
                    rb.set_ang_vel(new_ang);
                },
                SoftBody(_) => {
                    fail!("Not yet implemented.")
                }
            }
        }
    }

    #[inline]
    fn priority(&self) -> f64 { 100.0 }
}
