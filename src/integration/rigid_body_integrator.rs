use std::ptr;
use std::num::One;
use nalgebra::traits::transformation::{Transformation, Transform};
use nalgebra::traits::rotation::{Rotation, Rotate};
use nalgebra::traits::translation::{Translation, Translatable};
use nalgebra::traits::inv::Inv;
use nalgebra::traits::vector::Vec;
use ncollide::util::hash_map::HashMap;
use ncollide::util::hash::UintTWHash;
use object::body::ToRigidBody;
use object::rigid_body::RigidBody;
use object::volumetric::InertiaTensor;
use integration::integrator::Integrator;
use integration::euler;

pub struct RigidBodyExpEulerIntegrator<N, LV, AV, M, II> {
    priv objects: HashMap<uint, @mut RigidBody<N, LV, AV, M, II>, UintTWHash>,
}

impl<N, LV, AV, M, II> RigidBodyExpEulerIntegrator<N, LV, AV, M, II> {
    #[inline]
    pub fn new() -> RigidBodyExpEulerIntegrator<N, LV, AV, M, II> {
        RigidBodyExpEulerIntegrator {
            objects: HashMap::new(UintTWHash)
        }
    }
}

impl<N:  Clone,
     M:  Clone + Inv + Mul<M, M> + Rotation<AV> + Rotate<LV> + Translation<LV> +
         Translatable<LV, M> + Transform<LV> + One,
     LV: Clone + Vec<N>,
     AV: Clone + Vec<N>,
     II: Clone + Mul<II, II> + InertiaTensor<N, LV, M> + Inv,
     B: ToRigidBody<N, LV, AV, M, II>>
Integrator<N, B> for RigidBodyExpEulerIntegrator<N, LV, AV, M, II> {
    #[inline]
    fn add(&mut self, o: @mut B) {
        match o.to_rigid_body() {
            Some(rb) => { self.objects.insert(ptr::to_mut_unsafe_ptr(rb) as uint, rb); },
            None     => { }
        }
    }

    #[inline]
    fn remove(&mut self, o: @mut B) {
        match o.to_rigid_body() {
            Some(rb) => { self.objects.remove(&(ptr::to_mut_unsafe_ptr(rb) as uint)); },
            None     => { }
        }
    }

    #[inline]
    fn activate(&mut self, o: @mut B) {
        self.add(o);
    }

    #[inline]
    fn deactivate(&mut self, o: @mut B) {
        self.remove(o)
    }

    #[inline]
    fn pre_update(&mut self, dt: N) {
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
    fn post_update(&mut self, _: N) {
    }
}

pub struct RigidBodySmpEulerIntegrator<N, LV, AV, M, II> {
    priv objects: HashMap<uint, @mut RigidBody<N, LV, AV, M, II>, UintTWHash>,
}

impl<N, LV, AV, M, II> RigidBodySmpEulerIntegrator<N, LV, AV, M, II> {
    #[inline]
    pub fn new() -> RigidBodySmpEulerIntegrator<N, LV, AV, M, II> {
        RigidBodySmpEulerIntegrator {
            objects: HashMap::new(UintTWHash)
        }
    }
}

impl<N:  Clone,
     M:  Clone + Inv + Mul<M, M> + Rotation<AV> + Transform<LV> +
     Translation<LV> + Translatable<LV, M> + Rotate<LV> + One,
     LV: Clone + Vec<N>,
     AV: Clone + Vec<N>,
     II: Clone + Mul<II, II> + Inv + InertiaTensor<N, LV, M>,
     B: ToRigidBody<N, LV, AV, M, II>>
Integrator<N, B> for RigidBodySmpEulerIntegrator<N, LV, AV, M, II> {
    #[inline]
    fn add(&mut self, o: @mut B) {
        match o.to_rigid_body() {
            Some(rb) => { self.objects.insert(ptr::to_mut_unsafe_ptr(rb) as uint, rb); },
            None     => { }
        }
    }

    #[inline]
    fn remove(&mut self, o: @mut B) {
        match o.to_rigid_body() {
            Some(rb) => { self.objects.remove(&(ptr::to_mut_unsafe_ptr(rb) as uint)); },
            None     => { }
        }
    }

    #[inline]
    fn activate(&mut self, o: @mut B) {
        self.add(o);
    }

    #[inline]
    fn deactivate(&mut self, o: @mut B) {
        self.remove(o)
    }

    #[inline]
    fn pre_update(&mut self, _: N) {
    }

    #[inline]
    fn post_update(&mut self, dt: N) {
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
}
