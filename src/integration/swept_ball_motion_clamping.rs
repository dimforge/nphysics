use std::ptr;
use std::managed;
use nalgebra::na::{Translation, Norm};
use nalgebra::na;
use ncollide::bounding_volume::{AABB, BoundingVolume, ball_aabb};
use ncollide::util::hash_map::HashMap;
use ncollide::util::hash::UintTWHash;
use ncollide::geom::Ball;
use ncollide::broad::{RayCastBroadPhase, BoundingVolumeBroadPhase};
use ncollide::math::{Scalar, Vector};
use integration::Integrator;
use object::{RigidBody, RB, SB};
use signal::signal::{SignalEmiter, BodyActivationSignalHandler};

struct CCDBody {
    body:        @mut RigidBody,
    radius:      Scalar,
    sqthreshold: Scalar,
    last_pos:    Vector
}

impl CCDBody {
    pub fn new(body:        @mut RigidBody,
               radius:      Scalar,
               sqthreshold: Scalar,
               last_pos:    Vector)
               -> CCDBody {
        CCDBody {
            body:        body,
            radius:      radius,
            sqthreshold: sqthreshold,
            last_pos:    last_pos
        }
    }
}

pub struct SweptBallMotionClamping<BF> {
    priv objects:       HashMap<uint, CCDBody, UintTWHash>,
    priv iobjects:      HashMap<uint, CCDBody, UintTWHash>,
    priv broad_phase:   @mut BF,
    priv update_bf:     bool,
    priv interferences: ~[@mut RigidBody]
}

impl<BF: 'static + RayCastBroadPhase<RigidBody> + BoundingVolumeBroadPhase<RigidBody, AABB>>
SweptBallMotionClamping<BF> {
    pub fn new<C>(events:    &mut SignalEmiter<RigidBody, C>,
                  bf:        @mut BF,
                  update_bf: bool)
                  -> @mut SweptBallMotionClamping<BF> {
        let res = @mut SweptBallMotionClamping {
            objects:       HashMap::new(UintTWHash::new()),
            iobjects:      HashMap::new(UintTWHash::new()),
            broad_phase:   bf,
            update_bf:     update_bf,
            interferences: ~[]
        };

        events.add_body_activation_handler(
            ptr::to_unsafe_ptr(res) as uint,
            res as @mut BodyActivationSignalHandler<RigidBody, C>
        );

        res
    }

    // FIXME: implement remove_ccd_from

    pub fn add_ccd_to(&mut self,
                      body:                @mut RigidBody,
                      swept_sphere_radius: Scalar,
                      motion_thresold:     Scalar) {
        let key = ptr::to_unsafe_ptr(body) as uint;
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

    fn activate(&mut self, o: @mut RigidBody) {
        let key = ptr::to_unsafe_ptr(o) as uint;

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

    fn deactivate(&mut self, o: @mut RigidBody) {
        let key = ptr::to_unsafe_ptr(o) as uint;

        match self.objects.get_and_remove(&key) {
            Some(o) => {
                self.iobjects.insert(key, o.value);
            },
            None => { }
        }
    }
}

impl<BF: RayCastBroadPhase<RigidBody> + BoundingVolumeBroadPhase<RigidBody, AABB>> Integrator<RigidBody>
for SweptBallMotionClamping<BF> {
    fn add(&mut self, o: @mut RigidBody) {
        if self.update_bf {
            self.broad_phase.add(o);
        }
    }

    fn remove(&mut self, o: @mut RigidBody) {
        if self.update_bf {
            self.broad_phase.remove(o);
        }

        self.objects.remove(&(ptr::to_unsafe_ptr(o) as uint));
    }

    fn update(&mut self, _: Scalar) {
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
                        let begin      = ball_aabb(&o.value.last_pos, &ball.radius());
                        let end        = ball_aabb(&curr_pos, &ball.radius());
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
                        let mut min_toi: Scalar = Bounded::max_value();
                        let old_transform  = na::append_translation(rb.transform_ref(), &-movement);
                        let mut dir        = movement.clone();
                        let distance       = dir.normalize();

                        let _eps: Scalar = Float::epsilon();
                        for b in self.interferences.iter() {
                            if !managed::mut_ptr_eq(*b, o.value.body) {
                                match **b {
                                    RB(ref rb) => {
                                        /*
                                        let toi =
                                            toi::geom_geom(
                                                &old_transform,
                                                &dir,
                                                &distance,
                                                &ball,
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
                                        */
                                        fail!("Review toi computation dispatch.")
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

impl<BF: 'static + RayCastBroadPhase<RigidBody> + BoundingVolumeBroadPhase<RigidBody, AABB>, C>
BodyActivationSignalHandler<RigidBody, C> for SweptBallMotionClamping<BF> {
    fn handle_body_activated_signal(&mut self, b: @mut RigidBody, _: &mut ~[C]) {
        self.activate(b)
    }

    fn handle_body_deactivated_signal(&mut self, b: @mut RigidBody) {
        self.deactivate(b)
    }
}
