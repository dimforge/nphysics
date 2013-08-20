use std::ptr;
use nalgebra::traits::vector::Vec;
use ncollide::util::hash_map::HashMap;
use ncollide::util::hash::UintTWHash;
use object::body::ToSoftBody;
use object::soft_body::SoftBody;
use integration::integrator::Integrator;
use integration::euler;

pub struct SoftBodyExpEulerIntegrator<N, V> {
    priv objects:  HashMap<uint, @mut SoftBody<N, V>, UintTWHash>,
}

impl<N: Clone, V: Vec<N>, B: ToSoftBody<N, V>> Integrator<N, B>
for SoftBodyExpEulerIntegrator<N, V> {
    fn add(&mut self, o: @mut B) {
        match o.to_soft_body() {
            Some(sb) => { self.objects.insert(ptr::to_mut_unsafe_ptr(sb) as uint, sb); },
            None     => { }
        }
    }

    fn remove(&mut self, o: @mut B) {
        match o.to_soft_body() {
            Some(sb) => { self.objects.remove(&(ptr::to_mut_unsafe_ptr(sb) as uint)); },
            None     => { }
        }
    }

    fn activate(&mut self, o: @mut B) {
        self.add(o);
    }

    fn deactivate(&mut self, o: @mut B) {
        self.remove(o)
    }

    fn update(&mut self, dt: N) {
        for o in self.objects.elements().iter() {
            for pt in o.value.points.mut_iter() {
                let (p, v) = euler::explicit_integrate_wo_rotation(
                    dt.clone(),
                    &pt.position,
                    &pt.velocity,
                    &o.value.acc);

                pt.position = p;
                pt.velocity = v;
            }
        }
    }
}

pub struct SoftBodySmpEulerIntegrator<N, V> {
    priv objects:  HashMap<uint, @mut SoftBody<N, V>, UintTWHash>,
}

impl<N: Clone, V: Vec<N>, B: ToSoftBody<N, V>> Integrator<N, B>
for SoftBodySmpEulerIntegrator<N, V> {
    fn add(&mut self, o: @mut B) {
        match o.to_soft_body() {
            Some(sb) => { self.objects.insert(ptr::to_mut_unsafe_ptr(sb) as uint, sb); },
            None     => { }
        }
    }

    fn remove(&mut self, o: @mut B) {
        match o.to_soft_body() {
            Some(sb) => { self.objects.remove(&(ptr::to_mut_unsafe_ptr(sb) as uint)); },
            None     => { }
        }
    }

    fn activate(&mut self, o: @mut B) {
        self.add(o);
    }

    fn deactivate(&mut self, o: @mut B) {
        self.remove(o)
    }

    fn update(&mut self, dt: N) {
        for o in self.objects.elements().iter() {
            for pt in o.value.points.mut_iter() {
                let (p, v) = euler::semi_implicit_integrate_wo_rotation(
                    dt.clone(),
                    &pt.position,
                    &pt.velocity,
                    &o.value.acc
                    );

                pt.position = p;
                pt.velocity = v;
            }
        }
    }
}
