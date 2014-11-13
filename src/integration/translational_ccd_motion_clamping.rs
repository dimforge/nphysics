use na::Translation;
use na;
use ncollide::utils::data::has_uid::HasUid;
use ncollide::utils::data::hash_map::HashMap;
use ncollide::utils::data::hash::UintTWHash;
use ncollide::broad_phase::BroadPhase;
use ncollide::bounding_volume::{BoundingVolume, AABB};
use ncollide::geometry;
use object::RigidBodyHandle;
use math::{Scalar, Point, Vect};


struct CCDBody {
    body:        RigidBodyHandle,
    sqthreshold: Scalar,
    last_pos:    Vect,
    accept_zero: bool
}

impl CCDBody {
    fn new(body: RigidBodyHandle, threshold: Scalar) -> CCDBody {
        CCDBody {
            sqthreshold: threshold * threshold,
            last_pos:    body.borrow().translation(),
            body:        body,
            accept_zero: true
        }
    }
}

/// Handles Continuous Collision Detection.
pub struct TranslationalCCDMotionClamping {
    objects: HashMap<uint, CCDBody, UintTWHash>,
    interferences_collector: Vec<RigidBodyHandle>
}

impl TranslationalCCDMotionClamping {
    /// Creates a new `TranslationalCCDMotionClamping` to enable continuous collision detection to
    /// fast-moving rigid bodies.
    pub fn new() -> TranslationalCCDMotionClamping {
        TranslationalCCDMotionClamping {
            objects:                 HashMap::new(UintTWHash::new()),
            interferences_collector: Vec::new()
        }
    }

    /// Enables continuous collision for the given rigid body.
    pub fn add_ccd_to(&mut self, body: RigidBodyHandle, motion_threshold: Scalar) {
        self.objects.insert(body.uid(), CCDBody::new(body, motion_threshold));
    }

    /// Remove continuous collision from the given rigid body.
    pub fn remove_ccd_from(&mut self, body: &RigidBodyHandle) {
        self.objects.remove(&body.uid());
    }

    /// Update the time of impacts and apply motion clamping when necessary.
    pub fn update<BF, DV>(&mut self, bf: &mut BF)
        where BF: BroadPhase<Point, Vect, RigidBodyHandle, AABB<Point>, DV> {
        // XXX: we should no do this in a sequential order because CCD betwen two fast, CCD-enabled
        // objects, will not work properly (it will be biased toward the first object).
        for o in self.objects.elements_mut().iter_mut() {
            let brb1 = o.value.body.borrow();

            let movement = brb1.translation() - o.value.last_pos;

            if na::sqnorm(&movement) > o.value.sqthreshold {
                // Use CCD for this object.
                let last_transform = na::append_translation(brb1.transform_ref(), &-movement);
                let begin_aabb = brb1.shape_ref().aabb(&last_transform);
                let end_aabb   = brb1.shape_ref().aabb(brb1.transform_ref());
                let swept_aabb = begin_aabb.merged(&end_aabb);

                /*
                 * Ask the broad phase for interferences.
                 */
                // FIXME: performing a convex-cast here would be much more efficient.
                bf.interferences_with_bounding_volume(
                    &swept_aabb,
                    &mut self.interferences_collector);

                /*
                 * Find the minimum toi.
                 */
                let mut min_toi = na::one::<Scalar>();
                let mut toi_found = false;
                let dir = movement.clone();

                let _eps: Scalar = Float::epsilon();

                for rb2 in self.interferences_collector.iter() {
                    if rb2.uid() != o.value.body.uid() {
                        let brb2 = rb2.borrow();

                        let toi = geometry::time_of_impact_internal::shape_against_shape(
                                    &last_transform,
                                    &dir,
                                    brb1.shape_ref(),
                                    brb2.transform_ref(),
                                    &na::zero(), // assume the other object does not move.
                                    brb2.shape_ref());

                        match toi {
                            Some(t) => {
                                if t <= min_toi { // we need the equality case to set the `toi_found` flag.
                                    toi_found = true;

                                    if t > _eps || o.value.accept_zero {
                                        min_toi = t;
                                    }
                                }
                            },
                            None => { }
                        }
                    }
                }

                self.interferences_collector.clear();

                /*
                 * Revert the object translation at the toi
                 */
                drop(brb1);

                if toi_found {
                    o.value.body.borrow_mut().append_translation(&(-dir * (na::one::<Scalar>() - min_toi)));
                    o.value.accept_zero = false;
                }
                else {
                    o.value.accept_zero = true;
                }

                /*
                 * We moved the object: ensure the broad phase takes that in account.
                 */
                bf.update_object(&o.value.body);
            }

            o.value.last_pos = o.value.body.borrow().translation();
        }
    }
}
