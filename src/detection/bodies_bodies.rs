//! Collision detector between rigid bodies.

use std::cell::RefCell;
use std::rc::Rc;
use std::vec::Vec;
use ncollide::bounding_volume::{HasBoundingVolume, AABB};
use ncollide::broad::{Dispatcher, InterferencesBroadPhase, BoundingVolumeBroadPhase, RayCastBroadPhase};
use ncollide::narrow::{CollisionDetector, GeomGeomDispatcher, GeomGeomCollisionDetector};
use ncollide::contact::Contact;
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
    priv geom_dispatcher: Rc<GeomGeomDispatcher>
}

impl BodyBodyDispatcher {
    /// Creates a new `BodyBodyDispatcher` given a dispatcher for pairs of rigid bodies' geometry.
    pub fn new(d: Rc<GeomGeomDispatcher>) -> BodyBodyDispatcher {
        BodyBodyDispatcher {
            geom_dispatcher: d
        }
    }
}

impl Dispatcher<Rc<RefCell<RigidBody>>, Rc<RefCell<RigidBody>>, ~GeomGeomCollisionDetector> for BodyBodyDispatcher {
    fn dispatch(&self, rb1: &Rc<RefCell<RigidBody>>, rb2: &Rc<RefCell<RigidBody>>) -> ~GeomGeomCollisionDetector {
        let brb1 = rb1.borrow();
        let brb2 = rb2.borrow();

        self.geom_dispatcher.dispatch(brb1.get().geom(), brb2.get().geom())
    }

    fn is_valid(&self, a: &Rc<RefCell<RigidBody>>, b: &Rc<RefCell<RigidBody>>) -> bool {
        if &**a as *RefCell<RigidBody> == &**b as *RefCell<RigidBody> {
            false
        }
        else {
            let ba = a.borrow();
            let bb = b.borrow();

            ba.get().can_move() || bb.get().can_move()
        }
    }
}


/// Collision detector between rigid bodies.
pub struct BodiesBodies<BF> {
    priv geom_geom_dispatcher:  Rc<GeomGeomDispatcher>,
    priv contacts_collector:    Vec<Contact>,
    // FIXME: this is an useless buffer which accumulate the result of bodies activation.
    // This must exist since there is no way to send an activation message without an accumulation
    // list…
    priv constraints_collector: Vec<Constraint>,
}

impl<BF: 'static + InterferencesBroadPhase<Rc<RefCell<RigidBody>>, ~GeomGeomCollisionDetector>> BodiesBodies<BF> {
    /// Creates a new `BodiesBodies` collision detector.
    pub fn new(dispatcher: Rc<GeomGeomDispatcher>) -> BodiesBodies<BF> {
        BodiesBodies {
            geom_geom_dispatcher:  dispatcher,
            contacts_collector:    Vec::new(),
            constraints_collector: Vec::new(),
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

        for rb in bodies.move_iter() { // FIXME: no move_rev_iter?
            let toi;

            {
                toi = rb.get().geom().toi_with_transform_and_ray(rb.get().transform_ref(), ray, true)
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
        if !o.get().is_active() {
            // wake up everybody in contact
            let aabb              = o.get().bounding_volume();
            let mut interferences = Vec::new();

            broad_phase.interferences_with_bounding_volume(&aabb, &mut interferences);

            for i in interferences.iter() {
                if &**i as *RefCell<RigidBody> != &**o as *RefCell<RigidBody> &&
                   i.with(|i| !i.is_active() && i.can_move()) {
                    activation.will_activate(i);
                }
            }
        }
    }
}

impl<BF: InterferencesBroadPhase<Rc<RefCell<RigidBody>>, ~GeomGeomCollisionDetector> +
         BoundingVolumeBroadPhase<Rc<RefCell<RigidBody>>, AABB>>
Detector<RigidBody, Constraint, BF> for BodiesBodies<BF> {
    fn update(&mut self, broad_phase: &mut BF, activation: &mut ActivationManager) {
        broad_phase.for_each_pair_mut(|b1, b2, cd| {
            let ncols = cd.num_colls();

            {
                cd.update(&*self.geom_geom_dispatcher,
                          b1.get().transform_ref(),
                          b1.get().geom(),
                          b2.get().transform_ref(),
                          b2.get().geom());
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
                out.push(RBRB(b1.clone(), b2.clone(), c.clone()))
            }

            self.contacts_collector.clear()
        })
    }
}
