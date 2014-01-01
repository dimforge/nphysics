use std::ptr;
use std::borrow;
use std::managed;
use std::rc::Rc;
use ncollide::bounding_volume::{HasBoundingVolume, AABB};
use ncollide::broad::{Dispatcher, InterferencesBroadPhase, BoundingVolumeBroadPhase, RayCastBroadPhase};
use ncollide::narrow::{CollisionDetector, GeomGeomDispatcher, GeomGeomCollisionDetector};
use ncollide::contact::Contact;
use ncollide::ray::{Ray, RayCastWithTransform};
use ncollide::math::N;
use object::{Body, RB, SB};
use detection::constraint::{Constraint, RBRB};
use detection::detector::Detector;
use signal::signal::{SignalEmiter, BodyActivationSignalHandler};

pub struct BodyBodyDispatcher {
    // XXX: we use a MutexArc here since because of https://github.com/mozilla/rust/issues/9265
    // it is not possible to make the GeomGeomDispatcher Freeze.
    geom_dispatcher: Rc<GeomGeomDispatcher>
}

impl BodyBodyDispatcher {
    pub fn new(d: Rc<GeomGeomDispatcher>) -> BodyBodyDispatcher {
        BodyBodyDispatcher {
            geom_dispatcher: d
        }
    }
}

impl Dispatcher<Body, Body, ~GeomGeomCollisionDetector> for BodyBodyDispatcher {
    fn dispatch(&self, a: &Body, b: &Body)
        -> ~GeomGeomCollisionDetector {
        match (a, b) {
            (&RB(ref rb1), &RB(ref rb2)) => {
                self.geom_dispatcher.borrow().dispatch(rb1.geom(), rb2.geom())
            },
            _ => fail!("Not yet implemented")
        }
    }

    fn is_valid(&self,
                a: &Body,
                b: &Body)
                -> bool {
        if borrow::ref_eq(a, b) {
            return false
        }

        match (a, b) {
            (&RB(ref a), &RB(ref b)) => a.can_move() || b.can_move(),
            _ => true
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
    signals:     @mut SignalEmiter<Body, Constraint>,
    broad_phase: @mut BF,
    update_bf:   bool
}

impl<BF: 'static + InterferencesBroadPhase<Body, ~GeomGeomCollisionDetector>> BodiesBodies<BF> {
    pub fn new(events:     @mut SignalEmiter<Body, Constraint>,
               bf:         @mut BF,
               dispatcher: Rc<GeomGeomDispatcher>,
               update_bf:  bool) -> @mut BodiesBodies<BF> {
        let res = @mut BodiesBodies {
            geom_geom_dispatcher:  dispatcher,
            contacts_collector:    ~[],
            constraints_collector: ~[],
            signals:               events,
            broad_phase:           bf,
            update_bf:             update_bf
        };

        events.add_body_activation_handler(
            ptr::to_mut_unsafe_ptr(res) as uint,
            res as @mut BodyActivationSignalHandler<Body, Constraint>
        );

        res
    }

    fn activate(&mut self,
                body: @mut Body,
                out:  &mut ~[Constraint]) {
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

    fn deactivate(&mut self, body: @mut Body) {
        self.broad_phase.deactivate(body)
    }
}

impl<BF: RayCastBroadPhase<Body>>
BodiesBodies<BF> {
    pub fn interferences_with_ray(&mut self,
                                  ray: &Ray,
                                  out: &mut ~[(@mut Body, N)]) {
        let mut bodies = ~[];

        self.broad_phase.interferences_with_ray(ray, &mut bodies);

        for b in bodies.iter() {
            match **b {
                RB(ref rb) => {
                    match rb.geom().toi_with_transform_and_ray(rb.transform_ref(), ray) {
                        None    => { },
                        Some(t) => out.push((*b, t))
                    }
                },
                SB(_) => fail!("Not yet implemented.")
            }
        }
    }
}

impl<BF: InterferencesBroadPhase<Body, ~GeomGeomCollisionDetector> +
         BoundingVolumeBroadPhase<Body, AABB>>
Detector<Body, Constraint>
for BodiesBodies<BF> {
    fn add(&mut self, o: @mut Body) {
        if self.update_bf {
            self.broad_phase.add(o);
        }
    }

    fn remove(&mut self, o: @mut Body) {
        if !o.is_active() {
            // wake up everybody in contact
            let aabb              = o.bounding_volume();
            let mut interferences = ~[];

            // FIXME: change that to a collision lost event
            self.broad_phase.interferences_with_bounding_volume(&aabb, &mut interferences);

            for i in interferences.iter() {
                if !managed::mut_ptr_eq(o, *i) && !i.is_active() && i.can_move() {
                    self.signals.request_body_activation(*i);
                }
            }
        }

        // remove
        if self.update_bf {
            self.broad_phase.remove(o);
        }
    }

    fn update(&mut self) {
        if self.update_bf {
            self.broad_phase.update();
        }

        self.broad_phase.for_each_pair_mut(|b1, b2, cd| {
            let rb1 = b1.to_rigid_body_or_fail();
            let rb2 = b2.to_rigid_body_or_fail();

            let ncols = cd.num_colls();

            cd.update(self.geom_geom_dispatcher.borrow(),
                      rb1.transform_ref(),
                      rb1.geom(),
                      rb2.transform_ref(),
                      rb2.geom());

            let new_ncols = cd.num_colls();

            if ncols == 0 && new_ncols != 0 {
                // collision lost
                self.signals.emit_collision_started(b1, b2);
            }
            else if ncols != 0 && new_ncols == 0 {
                // collision created
                self.signals.emit_collision_ended(b1, b2);
            }
        })
    }

    fn interferences(&mut self, out: &mut ~[Constraint]) {
        self.broad_phase.for_each_pair_mut(|b1, b2, cd| {
            cd.colls(&mut self.contacts_collector);

            for c in self.contacts_collector.iter() {
                out.push(RBRB(b1, b2, c.clone()))
            }

            self.contacts_collector.clear()
        })
    }

    #[inline]
    fn priority(&self) -> f64 { 50.0 }
}

impl<BF: 'static + InterferencesBroadPhase<Body, ~GeomGeomCollisionDetector>>
BodyActivationSignalHandler<Body, Constraint> for BodiesBodies<BF> {
    fn handle_body_activated_signal(&mut self, b: @mut Body, out: &mut ~[Constraint]) {
        self.activate(b, out)
    }

    fn handle_body_deactivated_signal(&mut self, b: @mut Body) {
        self.deactivate(b)
    }
}
