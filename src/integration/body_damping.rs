use std::ptr;
use ncollide::util::hash_map::HashMap;
use ncollide::util::hash::UintTWHash;
use ncollide::math::N;
use object::{Body, RB, SB};
use integration::Integrator;
use signal::signal::{SignalEmiter, BodyActivationSignalHandler};

pub struct BodyDamping {
    priv linear_damping:  N,
    priv angular_damping: N,
    priv objects:         HashMap<uint, @mut Body, UintTWHash>
}

impl BodyDamping {
    #[inline]
    pub fn new<C>(events:          &mut SignalEmiter<Body, C>,
                  linear_damping:  N,
                  angular_damping: N)
                  -> @mut BodyDamping {
        let res = @mut BodyDamping {
            linear_damping:  linear_damping,
            angular_damping: angular_damping,
            objects:         HashMap::new(UintTWHash::new())
        };

        events.add_body_activation_handler(
            ptr::to_mut_unsafe_ptr(res) as uint,
            res as @mut BodyActivationSignalHandler<Body, C>
        );

        res
    }
}

impl Integrator<Body> for BodyDamping {
    #[inline]
    fn add(&mut self, o: @mut Body) {
        self.objects.insert(ptr::to_mut_unsafe_ptr(o) as uint, o);
    }

    #[inline]
    fn remove(&mut self, o: @mut Body) {
        self.objects.remove(&(ptr::to_mut_unsafe_ptr(o) as uint));
    }

    fn update(&mut self, _: N) {
        for o in self.objects.elements().iter() {
            match *o.value {
                RB(ref mut rb) => {
                    let new_lin = rb.lin_vel() * self.linear_damping;
                    rb.set_lin_vel(new_lin);
                    let new_ang = rb.ang_vel() * self.angular_damping;
                    rb.set_ang_vel(new_ang);
                },
                SB(_) => {
                    fail!("Not yet implemented.")
                }
            }
        }
    }

    #[inline]
    fn priority(&self) -> f64 { 100.0 }
}

impl<C>
BodyActivationSignalHandler<Body, C> for BodyDamping {
    fn handle_body_activated_signal(&mut self, b: @mut Body, _: &mut ~[C]) {
        self.add(b)
    }

    fn handle_body_deactivated_signal(&mut self, b: @mut Body) {
        self.remove(b)
    }
}
