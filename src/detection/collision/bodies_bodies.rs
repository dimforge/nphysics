use std::cell::RefCell;
use std::borrow;
use std::rc::Rc;
use ncollide::bounding_volume::AABB;
use ncollide::broad::{Dispatcher, InterferencesBroadPhase, BoundingVolumeBroadPhase, RayCastBroadPhase};
use ncollide::narrow::{CollisionDetector, GeomGeomDispatcher, GeomGeomCollisionDetector};
use ncollide::contact::Contact;
use ncollide::ray::{Ray, RayCastWithTransform};
use ncollide::math::N;
use object::RigidBody;
use detection::constraint::{Constraint, RBRB};
use detection::detector::Detector;

pub struct BodyBodyDispatcher {
    geom_dispatcher: Rc<GeomGeomDispatcher>
}

impl BodyBodyDispatcher {
    pub fn new(d: Rc<GeomGeomDispatcher>) -> BodyBodyDispatcher {
        BodyBodyDispatcher {
            geom_dispatcher: d
        }
    }
}

impl Dispatcher<Rc<RefCell<RigidBody>>, Rc<RefCell<RigidBody>>, ~GeomGeomCollisionDetector> for BodyBodyDispatcher {
    fn dispatch(&self, rb1: &Rc<RefCell<RigidBody>>, rb2: &Rc<RefCell<RigidBody>>) -> ~GeomGeomCollisionDetector {
        let brb1 = rb1.borrow().borrow();
        let brb2 = rb2.borrow().borrow();

        self.geom_dispatcher.borrow().dispatch(brb1.get().geom(), brb2.get().geom())
    }

    fn is_valid(&self, a: &Rc<RefCell<RigidBody>>, b: &Rc<RefCell<RigidBody>>) -> bool {
        if borrow::ref_eq(a.borrow(), b.borrow()) {
            false
        }
        else {
            let ba = a.borrow().borrow();
            let bb = b.borrow().borrow();

            ba.get().can_move() || bb.get().can_move()
        }
    }
}


pub struct BodiesBodies<BF> {
    geom_geom_dispatcher:  Rc<GeomGeomDispatcher>,
    contacts_collector:    ~[Contact],
    // FIXME: this is an useless buffer which accumulate the result of bodies activation.
    // This must exist since there is no way to send an activation message without an accumulation
    // list…
    constraints_collector: ~[Constraint],
}

impl<BF: 'static + InterferencesBroadPhase<Rc<RefCell<RigidBody>>, ~GeomGeomCollisionDetector>> BodiesBodies<BF> {
    pub fn new(dispatcher: Rc<GeomGeomDispatcher>) -> BodiesBodies<BF> {
        BodiesBodies {
            geom_geom_dispatcher:  dispatcher,
            contacts_collector:    ~[],
            constraints_collector: ~[],
        }
    }

    /* XXX: activation/deactivation
    fn activate(&mut self, body: @mut RigidBody, out:  &mut ~[Constraint]) {
        self.broad_phase.activate(body, |b1, b2, cd| {
            let rb1 = b1.to_rigid_body_or_fail();
            let rb2 = b2.to_rigid_body_or_fail();

            // FIXME: is the update needed? Or do we have enough guarantees to avoid it?
            cd.update(self.geom_geom_dispatcher.borrow(),
                      rb1.transform_ref(),
                      rb1.geom(),
                      rb2.transform_ref(),
                      rb2.geom());

            cd.colls(&mut self.contacts_collector);

            for c in self.contacts_collector.iter() {
                out.push(RBRB(b1, b2, c.clone()))
            }

            self.contacts_collector.clear()
        })
    }

    fn deactivate(&mut self, body: @mut RigidBody) {
        self.broad_phase.deactivate(body)
    }
    */
}

impl<BF: RayCastBroadPhase<Rc<RefCell<RigidBody>>>> BodiesBodies<BF> {
    pub fn interferences_with_ray(&mut self,
                                  ray:         &Ray,
                                  broad_phase: &mut BF,
                                  out:         &mut ~[(Rc<RefCell<RigidBody>>, N)]) {
        let mut bodies = ~[];

        broad_phase.interferences_with_ray(ray, &mut bodies);

        for rb in bodies.move_rev_iter() {
            let toi;

            {
                let brb = rb.borrow().borrow();

                toi = brb.get().geom().toi_with_transform_and_ray(brb.get().transform_ref(), ray)
            }

            match toi {
                None    => { },
                Some(t) => out.push((rb, t))
            }
        }
    }
}

impl<BF: InterferencesBroadPhase<Rc<RefCell<RigidBody>>, ~GeomGeomCollisionDetector> +
         BoundingVolumeBroadPhase<Rc<RefCell<RigidBody>>, AABB>>
Detector<RigidBody, Constraint, BF> for BodiesBodies<BF> {
    // XXX: deactivation/removal
    /*
    fn remove(&mut self, o: @mut RigidBody) {
        if !o.is_active() {
            // wake up everybody in contact
            let aabb              = o.bounding_volume();
            let mut interferences = ~[];

            // FIXME: change that to a collision lost event
            self.broad_phase.interferences_with_bounding_volume(&aabb, &mut interferences);

            for i in interferences.iter() {
                if !managed::mut_ptr_eq(o, *i) && !i.is_active() && i.can_move() {
                    println!(">>> request");
                    self.signals.request_body_activation(*i);
                    println!("<<< request");
                }
            }
        }

        // remove
        if self.update_bf {
            self.broad_phase.remove(o);
        }
    }
    */

    fn update(&mut self, broad_phase: &mut BF) {
        broad_phase.for_each_pair_mut(|b1, b2, cd| {
            let ncols = cd.num_colls();

            {
                let bb1 = b1.borrow().borrow();
                let bb2 = b2.borrow().borrow();
                let rb1 = bb1.get();
                let rb2 = bb2.get();

                cd.update(self.geom_geom_dispatcher.borrow(),
                          rb1.transform_ref(),
                          rb1.geom(),
                          rb2.transform_ref(),
                          rb2.geom());
            }

            let new_ncols = cd.num_colls();

            if ncols == 0 && new_ncols != 0 {
                // XXX: collision created
                // fail!("emit collision started");
                // self.signals.emit_collision_started(b1, b2);
            }
            else if ncols != 0 && new_ncols == 0 {
                // XXX: collision lost
                // fail!("emit collision ended");
                // self.signals.emit_collision_ended(b1, b2);
            }
        })
    }

    fn interferences(&mut self,
                     out:         &mut ~[Constraint],
                     broad_phase: &mut BF) {
        broad_phase.for_each_pair_mut(|b1, b2, cd| {
            cd.colls(&mut self.contacts_collector);

            for c in self.contacts_collector.iter() {
                out.push(RBRB(b1.clone(), b2.clone(), c.clone()))
            }

            self.contacts_collector.clear()
        })
    }
}
