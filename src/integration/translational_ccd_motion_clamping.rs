use std::cell::RefCell;
use na::Translation;
use na;
use ncollide::utils::data::hash_map::HashMap;
use ncollide::utils::data::hash::UintTWHash;
use ncollide::broad_phase::BroadPhase;
use ncollide::bounding_volume::BoundingVolume;
use ncollide::geometry;
use ncollide::bounding_volume;
use ncollide::math::FloatError;
use ncollide::world::CollisionGroups;
use world::RigidBodyCollisionWorld;
use object::RigidBodyHandle;
use object::RigidBody;
use math::{Scalar, Vect};


struct CCDBody {
    body:        RigidBodyHandle,
    sqthreshold: Scalar,
    last_pos:    Vect,
    accept_zero: bool
}

impl CCDBody {
    fn new(body: RigidBodyHandle, threshold: Scalar) -> CCDBody {
        let last_pos = body.borrow().position().translation();

        CCDBody {
            sqthreshold: threshold * threshold,
            last_pos:    last_pos,
            body:        body,
            accept_zero: true
        }
    }
}

/// Handles Continuous Collision Detection.
pub struct TranslationalCCDMotionClamping {
    objects: HashMap<usize, CCDBody, UintTWHash>
}

impl TranslationalCCDMotionClamping {
    /// Creates a new `TranslationalCCDMotionClamping` to enable continuous collision detection to
    /// fast-moving rigid bodies.
    pub fn new() -> TranslationalCCDMotionClamping {
        TranslationalCCDMotionClamping {
            objects: HashMap::new(UintTWHash::new())
        }
    }

    /// Enables continuous collision for the given rigid body.
    pub fn add_ccd_to(&mut self, body: RigidBodyHandle, motion_threshold: Scalar) {
        self.objects.insert(&*body as *const RefCell<RigidBody> as usize, CCDBody::new(body, motion_threshold));
    }

    /// Remove continuous collision from the given rigid body.
    pub fn remove_ccd_from(&mut self, body: &RigidBodyHandle) {
        self.objects.remove(&(&**body as *const RefCell<RigidBody> as usize));
    }

    /// Update the time of impacts and apply motion clamping when necessary.
    pub fn update(&mut self, cw: &mut RigidBodyCollisionWorld) {
        let mut update_collision_world = false;

        // XXX: we should no do this in a sequential order because CCD betwen two fast, CCD-enabled
        // objects, will not work properly (it will be biased toward the first object).
        for o in self.objects.elements_mut().iter_mut() {
            let brb1 = o.value.body.borrow();

            let movement = brb1.position().translation() - o.value.last_pos;

            if na::sqnorm(&movement) > o.value.sqthreshold {
                // Use CCD for this object.
                let last_transform = na::append_translation(brb1.position(), &-movement);
                let begin_aabb = bounding_volume::aabb(brb1.shape_ref(), &last_transform);
                let end_aabb   = bounding_volume::aabb(brb1.shape_ref(), brb1.position());
                let swept_aabb = begin_aabb.merged(&end_aabb);

                /*
                 * Find the minimum toi.
                 */
                let mut min_toi = na::one::<Scalar>();
                let mut toi_found = false;
                let dir = movement.clone();

                let _eps: Scalar = FloatError::epsilon();

                // FIXME: performing a convex-cast here would be much more efficient.
                let all_groups = CollisionGroups::new();
                for co2 in cw.interferences_with_aabb(&swept_aabb, &all_groups) {
                    if &*co2.data as *const RefCell<RigidBody> as usize !=
                       &*o.value.body as *const RefCell<RigidBody> as usize {
                        let brb2 = co2.data.borrow();

                        let toi = geometry::time_of_impact(
                            &last_transform,
                            &dir,
                            &*brb1.shape_ref(),
                            brb2.position(),
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
                cw.deferred_set_position(&* o.value.body as *const RefCell<RigidBody> as usize, o.value.body.borrow().position().clone());
                update_collision_world = true;
            }

            o.value.last_pos = o.value.body.borrow().position().translation();
        }

        if update_collision_world {
            cw.update();
        }
    }
}
