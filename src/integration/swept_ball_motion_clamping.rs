use std::num::One;
use std::ptr;
use std::managed;
use nalgebra::mat::{Translation, Rotate, Rotation, Transform, Inv};
use nalgebra::vec::{Vec, AlgebraicVecExt, Cross};
use ncollide::bounding_volume::{AABB, HasAABB, BoundingVolume};
use ncollide::util::hash_map::HashMap;
use ncollide::util::hash::UintTWHash;
use ncollide::geom::{Geom, Ball};
use ncollide::broad::{RayCastBroadPhase, BoundingVolumeBroadPhase};
use ncollide::narrow::toi;
use integration::Integrator;
use object::{Body, RB, SB};
use signal::signal::SignalEmiter;

struct CCDBody<N, LV, AV, M, II> {
    body:        @mut Body<N, LV, AV, M, II>,
    radius:      N,
    sqthreshold: N,
    last_pos:    LV
}

impl<N, LV, AV, M, II> CCDBody<N, LV, AV, M, II> {
    pub fn new(body:        @mut Body<N, LV, AV, M, II>,
               radius:      N,
               sqthreshold: N,
               last_pos:    LV)
               -> CCDBody<N, LV, AV, M, II> {
        CCDBody {
            body:        body,
            radius:      radius,
            sqthreshold: sqthreshold,
            last_pos:    last_pos
        }
    }
}

pub struct SweptBallMotionClamping<N, LV, AV, M, II, BF> {
    priv objects:       HashMap<uint, CCDBody<N, LV, AV, M, II>, UintTWHash>,
    priv iobjects:      HashMap<uint, CCDBody<N, LV, AV, M, II>, UintTWHash>,
    priv broad_phase:   @mut BF,
    priv update_bf:     bool,
    priv interferences: ~[@mut Body<N, LV, AV, M, II>]
}

impl<N:  'static + ApproxEq<N> + Num + Real + Float + Ord + Clone + ToStr + Algebraic,
     LV: 'static + AlgebraicVecExt<N> + Cross<AV> + ApproxEq<N> + Translation<LV> + Clone +
         Rotate<LV> + Transform<LV> + ToStr,
     AV: 'static + Vec<N> + ToStr,
     M:  'static + Rotation<AV> + Rotate<LV> + Translation<LV> + Transform<LV> + Mul<M, M> + Inv + One + ToStr,
     II: 'static + ToStr,
     BF: 'static + RayCastBroadPhase<LV, Body<N, LV, AV, M, II>> +
         BoundingVolumeBroadPhase<Body<N, LV, AV, M, II>, AABB<N, LV>>>
SweptBallMotionClamping<N, LV, AV, M, II, BF> {
    pub fn new<C>(events:    &mut SignalEmiter<N, Body<N, LV, AV, M, II>, C>,
                  bf:        @mut BF,
                  update_bf: bool)
                  -> @mut SweptBallMotionClamping<N, LV, AV, M, II, BF> {
        let res = @mut SweptBallMotionClamping {
            objects:       HashMap::new(UintTWHash::new()),
            iobjects:      HashMap::new(UintTWHash::new()),
            broad_phase:   bf,
            update_bf:     update_bf,
            interferences: ~[]
        };

        events.add_body_activated_handler(ptr::to_mut_unsafe_ptr(res) as uint, |o, _| res.activate(o));
        events.add_body_deactivated_handler(ptr::to_mut_unsafe_ptr(res) as uint, |o| res.deactivate(o));

        res
    }

    // FIXME: implement remove_ccd_from

    pub fn add_ccd_to(&mut self,
                      body:                @mut Body<N, LV, AV, M, II>,
                      swept_sphere_radius: N,
                      motion_thresold:     N) {
        let key = ptr::to_mut_unsafe_ptr(body) as uint;
        match *body {
            RB(ref mut rb) => {
                self.objects.insert(
                    key,
                    CCDBody::new(
                        body,
                        swept_sphere_radius,
                        motion_thresold * motion_thresold,
                        rb.transform_ref().translation()));
            },
            SB(_) => fail!("Soft bodies ccd is not yet implemented."),
        }
    }

    fn activate(&mut self, o: @mut Body<N, LV, AV, M, II>) {
        let key = ptr::to_mut_unsafe_ptr(o) as uint;

        match self.iobjects.get_and_remove(&key) {
            Some(entry) => {
                let mut ccdo = entry.value;

                match *o {
                    RB(ref rb) => ccdo.last_pos = rb.translation(),
                    SB(_)   => fail!("Not yet implemented.")
                }

                self.objects.insert(key, ccdo);
            },
            None => { }
        }
    }

    fn deactivate(&mut self, o: @mut Body<N, LV, AV, M, II>) {
        let key = ptr::to_mut_unsafe_ptr(o) as uint;

        match self.objects.get_and_remove(&key) {
            Some(o) => {
                self.iobjects.insert(key, o.value);
            },
            None => { }
        }
    }
}

impl<N:  ApproxEq<N> + Num + Real + Float + Ord + Clone + ToStr + Algebraic,
     LV: 'static + AlgebraicVecExt<N> + Cross<AV> + ApproxEq<N> + Translation<LV> + Clone +
         Rotate<LV> + Transform<LV> + ToStr,
     AV: Vec<N> + ToStr,
     M:  Rotation<AV> + Rotate<LV> + Translation<LV> + Transform<LV> + Mul<M, M> + Inv + One + ToStr,
     II: ToStr, // FIXME: remove those bounds
     BF: RayCastBroadPhase<LV, Body<N, LV, AV, M, II>> +
         BoundingVolumeBroadPhase<Body<N, LV, AV, M, II>, AABB<N, LV>>>
Integrator<N, Body<N, LV, AV, M, II>>
for SweptBallMotionClamping<N, LV, AV, M, II, BF> {
    fn add(&mut self, o: @mut Body<N, LV, AV, M, II>) {
        if self.update_bf {
            self.broad_phase.add(o);
        }
    }

    fn remove(&mut self, o: @mut Body<N, LV, AV, M, II>) {
        if self.update_bf {
            self.broad_phase.remove(o);
        }

        self.objects.remove(&(ptr::to_mut_unsafe_ptr(o) as uint));
    }

    fn update(&mut self, _: N) {
        if self.update_bf {
            self.broad_phase.update();
        }

        for o in self.objects.elements_mut().mut_iter() {
            match *o.value.body {
                RB(ref mut rb) => {
                    // FIXME: we dont put the sphere on the center of mass ?
                    let curr_pos = rb.translation();
                    let movement = curr_pos - o.value.last_pos;

                    if movement.sqnorm() > o.value.sqthreshold {
                        // activate CCD
                        /*
                         * Compute a bounding box enclosing the object motion
                         */
                        let ball       = Ball::new(o.value.radius.clone());
                        let begin      = ball.aabb(&o.value.last_pos);
                        let end        = ball.aabb(&curr_pos);
                        let dball      = Geom::new_ball(ball);
                        let swept_aabb = begin.merged(&end);

                        /*
                         * Ask the broad phase for interferences
                         */
                        self.broad_phase.interferences_with_bounding_volume(
                            &swept_aabb,
                            &mut self.interferences);

                        /*
                         * Find the minimum toi
                         */
                        let mut min_toi: N = Bounded::max_value();
                        let old_transform  = rb.transform_ref().translated(&-movement);
                        let mut dir        = movement.clone();
                        let distance       = dir.normalize();

                        let _eps: N = Float::epsilon();
                        for b in self.interferences.iter() {
                            if !managed::mut_ptr_eq(*b, o.value.body) {
                                match **b {
                                    RB(ref rb) => {
                                        let toi =
                                            toi::geom_geom(
                                                &old_transform,
                                                &dir,
                                                &distance,
                                                &dball,
                                                rb.transform_ref(),
                                                rb.geom());

                                        match toi {
                                            Some(ref t) => {
                                                if *t > _eps {
                                                    min_toi = min_toi.min(t)
                                                }
                                            },
                                            None        => { }
                                        }
                                    },
                                    SB(_) => fail!("Soft bodies are not yet supported.")
                                }
                            }
                        }

                        self.interferences.clear();

                        /*
                         * Revert the object translation at the toi
                         */
                        if min_toi < distance {
                            rb.translate_by(&(-dir * (distance - min_toi)));

                            /*
                             * We moved the object: ensure the broad phase notices that the object
                             */
                            self.broad_phase.update_object(o.value.body);
                        }
                    }

                    o.value.last_pos = rb.translation();
                },
                _ => fail!("CCD involving Soft bodies are not yet supported.")
            }
        }
    }

    #[inline]
    fn priority(&self) -> f64 { 100.0 }
}
