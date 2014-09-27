//! Collision detector between rigid bodies.

use std::cell::RefCell;
use std::rc::Rc;
use ncollide::bounding_volume::{HasBoundingVolume, AABB};
use ncollide::broad::{Dispatcher, InterferencesBroadPhase, BoundingVolumeBroadPhase, RayCastBroadPhase};
use ncollide::narrow::{CollisionDetector, GeomGeomDispatcher, GeomGeomCollisionDetector};
use ncollide::narrow::Contact;
use ncollide::ray::Ray;
use ncollide::math::Scalar;
use object::RigidBody;
use detection::constraint::{Constraint, RBRB};
use detection::detector::Detector;
use detection::activation_manager::ActivationManager;

/// Collision detector dispatcher for rigid bodies.
///
/// This is meat to be used as the broad phase collision dispatcher.
pub struct BodyBodyDispatcher {
    geom_dispatcher: Rc<GeomGeomDispatcher>
}

impl BodyBodyDispatcher {
    /// Creates a new `BodyBodyDispatcher` given a dispatcher for pairs of rigid bodies' geometry.
    pub fn new(d: Rc<GeomGeomDispatcher>) -> BodyBodyDispatcher {
        BodyBodyDispatcher {
            geom_dispatcher: d
        }
    }
}

impl Dispatcher<Rc<RefCell<RigidBody>>, Rc<RefCell<RigidBody>>, Box<GeomGeomCollisionDetector + Send>> for BodyBodyDispatcher {
    fn dispatch(&self, rb1: &Rc<RefCell<RigidBody>>, rb2: &Rc<RefCell<RigidBody>>) -> Option<Box<GeomGeomCollisionDetector + Send>> {
        let brb1 = rb1.borrow();
        let brb2 = rb2.borrow();

        self.geom_dispatcher.dispatch(brb1.geom_ref(), brb2.geom_ref())
    }

    fn is_valid(&self, a: &Rc<RefCell<RigidBody>>, b: &Rc<RefCell<RigidBody>>) -> bool {
        if &**a as *const RefCell<RigidBody> == &**b as *const RefCell<RigidBody> {
            false
        }
        else {
            let ba = a.borrow();
            let bb = b.borrow();

            ba.can_move() || bb.can_move()
        }
    }
}


/// Collision detector between rigid bodies.
pub struct BodiesBodies<BF> {
    geom_geom_dispatcher:  Rc<GeomGeomDispatcher>,
    contacts_collector:    Vec<Contact>,
}

impl<BF: InterferencesBroadPhase<Rc<RefCell<RigidBody>>, Box<GeomGeomCollisionDetector + Send>>> BodiesBodies<BF> {
    /// Creates a new `BodiesBodies` collision detector.
    pub fn new(dispatcher: Rc<GeomGeomDispatcher>) -> BodiesBodies<BF> {
        BodiesBodies {
            geom_geom_dispatcher:  dispatcher,
            contacts_collector:    Vec::new()
        }
    }
}

impl<BF: RayCastBroadPhase<Rc<RefCell<RigidBody>>>> BodiesBodies<BF> {
    /// Computes the interferences between every rigid bodies of a given broad phase, and a ray.
    pub fn interferences_with_ray(&mut self,
                                  ray:         &Ray,
                                  broad_phase: &mut BF,
                                  out:         &mut Vec<(Rc<RefCell<RigidBody>>, Scalar)>) {
        let mut bodies = Vec::new();

        broad_phase.interferences_with_ray(ray, &mut bodies);

        for rb in bodies.into_iter() { // FIXME: no move_rev_iter?
            let toi;

            {
                toi = rb.borrow().geom().toi_with_transform_and_ray(rb.borrow().transform_ref(), ray, true)
            }

            match toi {
                None    => { },
                Some(t) => out.push((rb, t))
            }
        }
    }
}

impl<BF: BoundingVolumeBroadPhase<Rc<RefCell<RigidBody>>, AABB>> BodiesBodies<BF> {
    /// Removes a rigid body from this detector.
    ///
    /// This must be called whenever a rigid body is removed from the physics world.
    pub fn remove(&mut self,
                  o:           &Rc<RefCell<RigidBody>>,
                  broad_phase: &mut BF,
                  activation:  &mut ActivationManager) {
        if !o.borrow().is_active() {
            // wake up everybody in contact
            let aabb              = o.borrow().bounding_volume();
            let mut interferences = Vec::new();

            broad_phase.interferences_with_bounding_volume(&aabb, &mut interferences);

            for i in interferences.iter() {
                let bi = i.borrow();
                if &**i as *const RefCell<RigidBody> != &**o as *const RefCell<RigidBody> &&
                   !bi.is_active() && bi.can_move() {
                        activation.will_activate(i);
                }
            }
        }
    }
}

impl<BF: InterferencesBroadPhase<Rc<RefCell<RigidBody>>, Box<GeomGeomCollisionDetector + Send>> +
         BoundingVolumeBroadPhase<Rc<RefCell<RigidBody>>, AABB>>
Detector<RigidBody, Constraint, BF> for BodiesBodies<BF> {
    fn update(&mut self, broad_phase: &mut BF, activation: &mut ActivationManager) {
        broad_phase.for_each_pair_mut(|b1, b2, cd| {
            let ncols = cd.num_colls();

            {
                cd.update(&*self.geom_geom_dispatcher,
                          b1.borrow().transform_ref(),
                          b1.borrow().geom_ref(),
                          b2.borrow().transform_ref(),
                          b2.borrow().geom_ref());
            }

            let new_ncols = cd.num_colls();

            if ncols == 0 && new_ncols != 0 {
                activation.will_activate(b1);
                activation.will_activate(b2);
            }
            else if ncols != 0 && new_ncols == 0 {
                activation.will_activate(b1);
                activation.will_activate(b2);
            }
        })
    }

    fn interferences(&mut self, out: &mut Vec<Constraint>, broad_phase: &mut BF) {
        broad_phase.for_each_pair_mut(|b1, b2, cd| {
            cd.colls(&mut self.contacts_collector);

            for c in self.contacts_collector.iter() {
                let mut contact = c.clone();
                // Take the margins in account.
                let m1 = b1.borrow().margin();
                let m2 = b2.borrow().margin();

                contact.depth = contact.depth + m1 + m2;

                out.push(RBRB(b1.clone(), b2.clone(), contact))
            }

            self.contacts_collector.clear()
        })
    }
}
