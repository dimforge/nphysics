use std::ptr;
use ncollide::util::hash_map::HashMap;
use ncollide::util::hash::UintTWHash;
use ncollide::math::{N, LV, AV};
use object::{Body, RB, SB};
use integration::Integrator;
use signal::signal::{SignalEmiter, BodyActivationSignalHandler};

// FIXME: split this on `RigidBodyForceGenerator` and `SoftBodyForceGenerator` ?
pub struct BodyForceGenerator {
    priv objects: HashMap<uint, @mut Body, UintTWHash>,
    priv lin_acc: LV,
    priv ang_acc: AV
}

impl BodyForceGenerator {
    pub fn new<C>(events:  &mut SignalEmiter<Body, C>,
                  lin_acc: LV,
                  ang_acc: AV)
                  -> @mut BodyForceGenerator {
        let res = @mut BodyForceGenerator {
            objects: HashMap::new(UintTWHash::new()),
            lin_acc: lin_acc,
            ang_acc: ang_acc
        };

        events.add_body_activation_handler(
            ptr::to_mut_unsafe_ptr(res) as uint,
            res as @mut BodyActivationSignalHandler<Body, C>
        );

        res
    }
}

impl BodyForceGenerator {
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
    fn write_accs_to(&self, o: &mut Body) {
        match *o {
            RB(ref mut rb) => {
                rb.set_lin_acc(self.lin_acc.clone());
                rb.set_ang_acc(self.ang_acc.clone());
            },
            SB(ref mut sb) => sb.acc = self.lin_acc.clone()
        }
    }
}

impl Integrator<Body> for BodyForceGenerator {
    #[inline]
    fn add(&mut self, o: @mut Body) {
        self.objects.insert(ptr::to_mut_unsafe_ptr(o) as uint, o);

        self.write_accs_to(o)
    }

    #[inline]
    fn remove(&mut self, o: @mut Body) {
        self.objects.remove(&(ptr::to_mut_unsafe_ptr(o) as uint));
    }

    #[inline]
    fn update(&mut self, _: N) { }

    #[inline]
    fn priority(&self) -> f64 { 0.0 }
}

impl<C>
BodyActivationSignalHandler<Body, C> for BodyForceGenerator {
    fn handle_body_activated_signal(&mut self, b: @mut Body, _: &mut ~[C]) {
        self.add(b)
    }

    fn handle_body_deactivated_signal(&mut self, b: @mut Body) {
        self.remove(b)
    }
}
