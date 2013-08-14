use std::ptr;
use ncollide::util::hash_map::HashMap;
use ncollide::util::hash::UintTWHash;
use object::body::{Body, RigidBody, SoftBody};
use integration::integrator::Integrator;

// FIXME: split this on `RigidBodyForceGenerator` and `SoftBodyForceGenerator` ?
pub struct BodyForceGenerator<N, LV, AV, M, II> {
    priv objects: HashMap<uint, @mut Body<N, LV, AV, M, II>, UintTWHash>,
    priv lin_acc: LV,
    priv ang_acc: AV
}

impl<N, LV, AV, M, II> BodyForceGenerator<N, LV, AV, M, II> {
    pub fn new(lin_acc: LV, ang_acc: AV) -> BodyForceGenerator<N, LV, AV, M, II> {
        BodyForceGenerator {
            objects: HashMap::new(UintTWHash),
            lin_acc: lin_acc,
            ang_acc: ang_acc
        }
    }
}

impl<N, M, LV: Clone, AV: Clone, II> BodyForceGenerator<N, LV, AV, M, II> {
    #[inline]
    pub fn lin_acc(&self) -> LV {
        self.lin_acc.clone()
    }

    #[inline]
    pub fn set_lin_acc(&mut self, lin_acc: LV) {
        self.lin_acc = lin_acc;

        for o in self.objects.elements().iter() {
            self.write_accs_to(o.value)
        }
    }

    #[inline]
    pub fn ang_acc(&self) -> AV {
        self.ang_acc.clone()
    }

    #[inline]
    pub fn set_ang_acc(&mut self, ang_acc: AV) {
        self.ang_acc = ang_acc;

        for o in self.objects.elements().iter() {
            self.write_accs_to(o.value)
        }
    }

    #[inline]
    fn write_accs_to(&self, o: &mut Body<N, LV, AV, M, II>) {
        match *o {
            RigidBody(rb) => {
                rb.set_lin_acc(self.lin_acc.clone());
                rb.set_ang_acc(self.ang_acc.clone());
            },
            SoftBody(sb) => sb.acc = self.lin_acc.clone()
        }
    }
}

impl<N: Clone, M: Clone, LV: Clone, AV: Clone, II: Clone> Integrator<N, Body<N, LV, AV, M, II>>
for BodyForceGenerator<N, LV, AV, M, II> {
    #[inline]
    fn add(&mut self, o: @mut Body<N, LV, AV, M, II>) {
        self.objects.insert(ptr::to_mut_unsafe_ptr(o) as uint, o);

        self.write_accs_to(o)
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

    #[inline]
    fn pre_update(&mut self, _: N) { }

    #[inline]
    fn post_update(&mut self, _: N) { }
}
