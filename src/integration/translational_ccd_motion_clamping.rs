use std::cell::RefCell;
use na;
use alga::general::Real;
use ncollide::utils::data::hash_map::HashMap;
use ncollide::utils::data::hash::UintTWHash;
use ncollide::bounding_volume::BoundingVolume;
use ncollide::query;
use ncollide::bounding_volume;
use ncollide::world::CollisionGroups;
use world::RigidBodyCollisionWorld;
use object::{RigidBodyHandle, SensorHandle, RigidBody};
use math::{Point, Translation};


struct CCDRigidBody<N: Real> {
    rigid_body:      RigidBodyHandle<N>,
    sqthreshold:     N,
    last_center:     Point<N>,
    trigger_sensors: bool,
    accept_zero:     bool
}

impl<N: Real> CCDRigidBody<N> {
    fn new(rigid_body: RigidBodyHandle<N>, threshold: N, trigger_sensors: bool) -> CCDRigidBody<N> {
        let last_center = rigid_body.borrow().position_center();

        CCDRigidBody {
            sqthreshold:     threshold * threshold,
            last_center:     last_center,
            rigid_body:      rigid_body,
            trigger_sensors: trigger_sensors,
            accept_zero:     true
        }
    }
}

/// Handles Continuous Collision Detection.
pub struct TranslationalCCDMotionClamping<N: Real> {
    objects: HashMap<usize, CCDRigidBody<N>, UintTWHash>,
    intersected_sensors_cache: Vec<(N, SensorHandle<N>)>
}

impl<N: Real> TranslationalCCDMotionClamping<N> {
    /// Creates a new `TranslationalCCDMotionClamping` to enable continuous collision detection to
    /// fast-moving rigid bodies.
    pub fn new() -> TranslationalCCDMotionClamping<N> {
        TranslationalCCDMotionClamping {
            objects:                   HashMap::new(UintTWHash::new()),
            intersected_sensors_cache: Vec::new()
        }
    }

    /// Enables continuous collision for the given rigid body.
    pub fn add_ccd_to(&mut self,
                      rigid_body:       RigidBodyHandle<N>,
                      motion_threshold: N,
                      trigger_sensors:  bool) {
        self.objects.insert(&*rigid_body as *const RefCell<RigidBody<N>> as usize,
                            CCDRigidBody::new(rigid_body, motion_threshold, trigger_sensors));
    }

    /// Enables continuous collision for the given rigid body.
    pub fn remove_ccd_from(&mut self, rigid_body: &RigidBodyHandle<N>) {
        self.objects.remove(&(&**rigid_body as *const RefCell<RigidBody<N>> as usize));
    }

    /// Update the time of impacts and apply motion clamping when necessary.
    ///
    /// Returns `false` if no clamping was done. If at least one clamping was performed, the
    /// collision word will be updated by this method once all the clamping have been performed.
    pub fn update(&mut self, cw: &mut RigidBodyCollisionWorld<N>) -> bool {
        let mut update_collision_world = false;

        // XXX: we should no do this in a sequential order because CCD between two fast
        // CCD-enabled objects will not work properly (it will be biased toward the first object).
        for co1 in self.objects.elements_mut().iter_mut() {
            let mut obj1 = co1.value.rigid_body.borrow_mut();

            let movement = obj1.position_center() - co1.value.last_center;

            if na::norm_squared(&movement) > co1.value.sqthreshold {
                // Use CCD for this object.
                let obj1_uid = &*co1.value.rigid_body as *const RefCell<RigidBody<N>> as usize;

                let last_transform = Translation::from_vector(-movement) * obj1.position();
                let begin_aabb     = bounding_volume::aabb(obj1.shape().as_ref(), &last_transform);
                let end_aabb       = bounding_volume::aabb(obj1.shape().as_ref(), obj1.position());
                let swept_aabb     = begin_aabb.merged(&end_aabb);

                /*
                 * Find the minimum TOI.
                 */
                let mut min_toi   = na::one::<N>();
                let mut toi_found = false;
                let dir = movement.clone();

                let _eps = N::default_epsilon();

                // XXX: handle groups.
                let all_groups = CollisionGroups::new();

                // FIXME: performing a convex-cast here would be much more efficient.
                for co2 in cw.interferences_with_aabb(&swept_aabb, &all_groups) {
                    if co2.data.uid() != obj1_uid {
                        let obj2 = co2.data.borrow();

                        let toi = query::time_of_impact(
                            &last_transform,
                            &dir,
                            obj1.shape().as_ref(),
                            &obj2.position(),
                            &na::zero(), // Assume the other object does not move.
                            obj2.shape().as_ref());

                        match toi {
                            Some(t) => {
                                if obj2.is_sensor() {
                                    if co1.value.trigger_sensors {
                                        self.intersected_sensors_cache.push((t, co2.data.clone().unwrap_sensor()));
                                        unimplemented!();
                                    }
                                }
                                else if t <= min_toi { // we need the equality case to set the `toi_found` flag.
                                    toi_found = true;

                                    if t > _eps || co1.value.accept_zero {
                                        min_toi = t;
                                    }
                                }
                            },
                            None => { }
                        }
                    }
                }

                /*
                 * Revert the object translation at the toi.
                 */
                if toi_found {
                    obj1.append_translation(&Translation::from_vector(-dir * (na::one::<N>() - min_toi)));
                    co1.value.accept_zero = false;

                    // We moved the object: ensure the broad phase takes that in account.
                    cw.deferred_set_position(obj1_uid, obj1.position().clone());
                    update_collision_world = true;
                }
                else {
                    co1.value.accept_zero = true;
                }

                /*
                 FIXME: * Activate then deactivate all the sensors that should have been traversed by the
                 * rigid body (we do not activate those that the rigid body entered without
                 * leaving).
                 */
                // self.intersected_sensors_cache.sort();
                // for sensor in self.intersected_sensors_cache.iter() {
                //     if sensor.0 < min_toi {
                //         let bsensor = sensor.borrow();

                //         // See if at the final rigid body position the sensor is still intersected.
                //         // NOTE: we are assuming the tensor is convex (handling concave cases would
                //         // be to complicated without much uses).
                //         if !query::test_interference(
                //             obj1.position(),
                //             obj1_shape,
                //             bsensor.position(),
                //             bsensor.shape_ref()) {
                //             // FIXME: activate the collision-start and collision-end signals for
                //             // this sensor.
                //         }
                //         // Otherwise do not trigger this sensor just yet. This will be done during
                //         // the next narrow phase update.
                //     }
                // }
            }

            co1.value.last_center = obj1.position_center();
            self.intersected_sensors_cache.clear();
        }

        if update_collision_world {
            cw.update();
            true
        }
        else {
            false
        }
    }
}
