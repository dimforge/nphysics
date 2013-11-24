use std::ptr;
use std::managed;
use nalgebra::na::Translation;
use nalgebra::na;
use ncollide::bounding_volume::{AABB, HasAABB, BoundingVolume};
use ncollide::util::hash_map::HashMap;
use ncollide::util::hash::UintTWHash;
use ncollide::geom::{Geom, Ball};
use ncollide::broad::{RayCastBroadPhase, BoundingVolumeBroadPhase};
use ncollide::narrow::toi;
use integration::Integrator;
use object::{Body, RB, SB};
use signal::signal::{SignalEmiter, BodyActivationSignalHandler};
use aliases::traits::{NPhysicsScalar, NPhysicsDirection, NPhysicsOrientation, NPhysicsTransform, NPhysicsInertia};

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

impl<N:  'static + Clone + NPhysicsScalar,
     LV: 'static + Clone + NPhysicsDirection<N, AV>,
     AV: 'static + Clone + NPhysicsOrientation<N>,
     M:  'static + Clone + NPhysicsTransform<LV, AV>,
     II: 'static + Clone + NPhysicsInertia<N, LV, AV, M>,
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

        events.add_body_activation_handler(
            ptr::to_mut_unsafe_ptr(res) as uint,
            res as @mut BodyActivationSignalHandler<Body<N, LV, AV, M, II>, C>
        );

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

impl<N:  Clone + NPhysicsScalar,
     LV: Clone + NPhysicsDirection<N, AV>,
     AV: Clone + NPhysicsOrientation<N>,
     M:  Clone + NPhysicsTransform<LV, AV>,
     II: Clone + NPhysicsInertia<N, LV, AV, M>,
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

                    if na::sqnorm(&movement) > o.value.sqthreshold {
                        // activate CCD
                        /*
                         * Compute a bounding box enclosing the object motion
                         */
                        let ball       = Ball::new(o.value.radius.clone());
                        let begin      = ball.aabb(&o.value.last_pos);
                        let end        = ball.aabb(&curr_pos);
                        let dball      = Geom::new_ball(ball.radius());
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
                        let old_transform  = na::append_translation(rb.transform_ref(), &-movement);
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
                            rb.append_translation(&(-dir * (distance - min_toi)));

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

impl<N:  'static + Clone + NPhysicsScalar,
     LV: 'static + Clone + NPhysicsDirection<N, AV>,
     AV: 'static + Clone + NPhysicsOrientation<N>,
     M:  'static + Clone + NPhysicsTransform<LV, AV>,
     II: 'static + Clone + NPhysicsInertia<N, LV, AV, M>,
     BF: 'static + RayCastBroadPhase<LV, Body<N, LV, AV, M, II>> +
         BoundingVolumeBroadPhase<Body<N, LV, AV, M, II>, AABB<N, LV>>,
     C>
BodyActivationSignalHandler<Body<N, LV, AV, M, II>, C> for SweptBallMotionClamping<N, LV, AV, M, II, BF> {
    fn handle_body_activated_signal(&mut self, b: @mut Body<N, LV, AV, M, II>, _: &mut ~[C]) {
        self.activate(b)
    }

    fn handle_body_deactivated_signal(&mut self, b: @mut Body<N, LV, AV, M, II>) {
        self.deactivate(b)
    }
}
