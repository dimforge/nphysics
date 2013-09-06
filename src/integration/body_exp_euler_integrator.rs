use std::ptr;
use std::num::One;
use nalgebra::mat::{Translation, Rotation, Rotate, Transformation, Transform, Inv};
use nalgebra::vec::Vec;
use ncollide::util::hash_map::HashMap;
use ncollide::util::hash::UintTWHash;
use object::{RB, SB};
use object::Body;
use object::volumetric::InertiaTensor;
use integration::Integrator;
use integration::euler;
use signal::signal::SignalEmiter;

pub struct BodyExpEulerIntegrator<N, LV, AV, M, II> {
    priv objects: HashMap<uint, @mut Body<N, LV, AV, M, II>, UintTWHash>,
}

impl<N:  'static + Clone,
     M:  'static + Clone + Inv + Mul<M, M> + Rotation<AV> + Rotate<LV> + Translation<LV> +
         Transform<LV> + One + ToStr,
     LV: 'static + Clone + Vec<N> + ToStr,
     AV: 'static + Clone + Vec<N> + ToStr,
     II: 'static + Clone + Mul<II, II> + InertiaTensor<N, LV, M> + Inv>
BodyExpEulerIntegrator<N, LV, AV, M, II> {
    #[inline]
    pub fn new<C>(events: &mut SignalEmiter<N, Body<N, LV, AV, M, II>, C>)
                  -> @mut BodyExpEulerIntegrator<N, LV, AV, M, II> {
        let res = @mut BodyExpEulerIntegrator {
            objects: HashMap::new(UintTWHash::new())
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
Integrator<N, Body<N, LV, AV, M, II>> for BodyExpEulerIntegrator<N, LV, AV, M, II> {
    #[inline]
    fn add(&mut self, o: @mut Body<N, LV, AV, M, II>) {
        self.objects.insert(ptr::to_mut_unsafe_ptr(o) as uint, o);
    }

    #[inline]
    fn remove(&mut self, o: @mut Body<N, LV, AV, M, II>) {
        self.objects.remove(&(ptr::to_mut_unsafe_ptr(o) as uint));
    }

    #[inline]
    fn update(&mut self, dt: N) {
        for o in self.objects.elements().iter() {
            match *o.value {
                RB(ref mut rb) => {
                    if rb.can_move() {
                        let (t, lv, av) = euler::explicit_integrate(
                            dt.clone(),
                            rb.transform_ref(),
                            rb.center_of_mass(),
                            &rb.lin_vel(),
                            &rb.ang_vel(),
                            &rb.lin_acc(),
                            &rb.ang_acc());

                        rb.transform_by(&t);
                        rb.set_lin_vel(lv);
                        rb.set_ang_vel(av);
                    }
                },
                SB(_) => {
                    fail!("Not yet implemented.")
                }
            }
        }
    }

    #[inline]
    fn priority(&self) -> f64 { 50.0 }
}
