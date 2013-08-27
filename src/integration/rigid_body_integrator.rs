use std::ptr;
use std::num::One;
use nalgebra::traits::transformation::{Transformation, Transform};
use nalgebra::traits::rotation::{Rotation, Rotate};
use nalgebra::traits::translation::Translation;
use nalgebra::traits::inv::Inv;
use nalgebra::traits::vector::Vec;
use ncollide::util::hash_map::HashMap;
use ncollide::util::hash::UintTWHash;
use object::body::{Body, RigidBody, SoftBody};
use object::rigid_body::RigidBody;
use object::volumetric::InertiaTensor;
use integration::integrator::Integrator;
use integration::euler;
use signal::signal::SignalEmiter;

pub struct RigidBodyExpEulerIntegrator<N, LV, AV, M, II> {
    priv objects: HashMap<uint, @mut RigidBody<N, LV, AV, M, II>, UintTWHash>,
}

impl<N:  'static + Clone,
     M:  'static + Clone + Inv + Mul<M, M> + Rotation<AV> + Rotate<LV> + Translation<LV> +
         Transform<LV> + One + ToStr,
     LV: 'static + Clone + Vec<N> + ToStr,
     AV: 'static + Clone + Vec<N> + ToStr,
     II: 'static + Clone + Mul<II, II> + InertiaTensor<N, LV, M> + Inv>
RigidBodyExpEulerIntegrator<N, LV, AV, M, II> {
    #[inline]
    pub fn new<C>(events: &mut SignalEmiter<N, Body<N, LV, AV, M, II>, C>)
                  -> @mut RigidBodyExpEulerIntegrator<N, LV, AV, M, II> {
        let res = @mut RigidBodyExpEulerIntegrator {
            objects: HashMap::new(UintTWHash)
        };

        events.add_body_activated_handler(ptr::to_mut_unsafe_ptr(res) as uint, |o, _| res.add(o));
        events.add_body_deactivated_handler(ptr::to_mut_unsafe_ptr(res) as uint, |o| res.remove(o));

        res
    }
}

impl<N:  Clone,
     M:  Clone + Inv + Mul<M, M> + Rotation<AV> + Rotate<LV> + Translation<LV> + Transform<LV> + One + ToStr,
     LV: Clone + Vec<N> + ToStr,
     AV: Clone + Vec<N> + ToStr,
     II: Clone + Mul<II, II> + InertiaTensor<N, LV, M> + Inv>
Integrator<N, Body<N, LV, AV, M, II>> for RigidBodyExpEulerIntegrator<N, LV, AV, M, II> {
    #[inline]
    fn add(&mut self, o: @mut Body<N, LV, AV, M, II>) {
        match *o {
            RigidBody(rb) => { self.objects.insert(ptr::to_mut_unsafe_ptr(rb) as uint, rb); },
            SoftBody(_)   => { }
        }
    }

    #[inline]
    fn remove(&mut self, o: @mut Body<N, LV, AV, M, II>) {
        match *o {
            RigidBody(rb) => { self.objects.remove(&(ptr::to_mut_unsafe_ptr(rb) as uint)); },
            SoftBody(_)   => { }
        }
    }

    #[inline]
    fn update(&mut self, dt: N) {
        for o in self.objects.elements().iter() {
            if o.value.can_move() {
                let (t, lv, av) = euler::explicit_integrate(
                    dt.clone(),
                    o.value.transform_ref(),
                    o.value.center_of_mass(),
                    &o.value.lin_vel(),
                    &o.value.ang_vel(),
                    &o.value.lin_acc(),
                    &o.value.ang_acc());

                o.value.transform_by(&t);
                o.value.set_lin_vel(lv);
                o.value.set_ang_vel(av);
            }
        }
    }

    #[inline]
    fn priority(&self) -> f64 { 50.0 }
}

pub struct RigidBodySmpEulerIntegrator<N, LV, AV, M, II> {
    priv objects: HashMap<uint, @mut RigidBody<N, LV, AV, M, II>, UintTWHash>,
}

impl<N:  'static + Clone,
     M:  'static + Clone + Inv + Mul<M, M> + Rotation<AV> + Transform<LV> + Translation<LV> +
         Rotate<LV> + One + ToStr,
     LV: 'static + Clone + Vec<N> + ToStr,
     AV: 'static + Clone + Vec<N> + ToStr,
     II: 'static + Clone + Mul<II, II> + Inv + InertiaTensor<N, LV, M>>
RigidBodySmpEulerIntegrator<N, LV, AV, M, II> {
    #[inline]
    pub fn new<C>(events: &mut SignalEmiter<N, Body<N, LV, AV, M, II>, C>)
                  -> @mut RigidBodySmpEulerIntegrator<N, LV, AV, M, II> {
        let res = @mut RigidBodySmpEulerIntegrator {
            objects: HashMap::new(UintTWHash)
        };

        events.add_body_activated_handler(ptr::to_mut_unsafe_ptr(res) as uint, |o, _| res.add(o));
        events.add_body_deactivated_handler(ptr::to_mut_unsafe_ptr(res) as uint, |o| res.remove(o));

        res
    }
}

impl<N:  Clone,
     M:  Clone + Inv + Mul<M, M> + Rotation<AV> + Transform<LV> + Translation<LV> + Rotate<LV> + One +
         ToStr,
     LV: Clone + Vec<N> + ToStr,
     AV: Clone + Vec<N> + ToStr,
     II: Clone + Mul<II, II> + Inv + InertiaTensor<N, LV, M>>
Integrator<N, Body<N, LV, AV, M, II>> for RigidBodySmpEulerIntegrator<N, LV, AV, M, II> {
    #[inline]
    fn add(&mut self, o: @mut Body<N, LV, AV, M, II>) {
        match *o {
            RigidBody(rb) => { self.objects.insert(ptr::to_mut_unsafe_ptr(rb) as uint, rb); },
            SoftBody(_)   => { }
        }
    }

    #[inline]
    fn remove(&mut self, o: @mut Body<N, LV, AV, M, II>) {
        match *o {
            RigidBody(rb) => { self.objects.remove(&(ptr::to_mut_unsafe_ptr(rb) as uint)); },
            SoftBody(_)   => { }
        }
    }

    #[inline]
    fn update(&mut self, dt: N) {
        for o in self.objects.elements().iter() {
            if o.value.can_move() {
                let (t, lv, av) = euler::semi_implicit_integrate(
                    dt.clone(),
                    o.value.transform_ref(),
                    o.value.center_of_mass(),
                    &o.value.lin_vel(),
                    &o.value.ang_vel(),
                    &o.value.lin_acc(),
                    &o.value.ang_acc());

                o.value.transform_by(&t);
                o.value.set_lin_vel(lv);
                o.value.set_ang_vel(av);
            }
        }
    }

    #[inline]
    fn priority(&self) -> f64 { 50.0 }
}
