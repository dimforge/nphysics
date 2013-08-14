use std::num::{Zero, One};
use std::borrow;
use nalgebra::traits::inv::Inv;
use nalgebra::traits::basis::Basis;
use nalgebra::traits::dim::Dim;
use nalgebra::traits::division_ring::DivisionRing;
use nalgebra::traits::dot::Dot;
use nalgebra::traits::cross::Cross;
use nalgebra::traits::norm::Norm;
use nalgebra::traits::translation::{Translation, Translatable};
use nalgebra::traits::rotation::{Rotate, Rotation};
use nalgebra::traits::sample::UniformSphereSample;
use nalgebra::traits::scalar_op::{ScalarAdd, ScalarSub, ScalarDiv, ScalarMul};
use nalgebra::traits::sub_dot::SubDot;
use nalgebra::traits::transformation::Transform;
use nalgebra::traits::vector_space::VectorSpace;
use ncollide::geom::minkowski_sum::AnnotatedPoint;
use ncollide::contact::Contact;
use ncollide::broad::dispatcher;
use ncollide::broad::dbvt_broad_phase::DBVTBroadPhase;
use ncollide::narrow::algorithm::johnson_simplex::{RecursionTemplate, JohnsonSimplex};
use ncollide::narrow::collision_detector::CollisionDetector;
use ncollide::bounding_volume::aabb::AABB;
use object::body::{Body, ToRigidBody, RigidBody};
use detection::detector::Detector;
use detection::collision::default_default::DefaultDefault;

enum PairwiseDetector<N, LV, AV, M, II> {
    RB(DefaultDefault<N, LV, AV, M, II>),
    Unsuported
}

// XXX: move this on its own file
pub enum Constraint<N, LV, AV, M, II> {
    RBRB(@mut Body<N, LV, AV, M, II>, @mut Body<N, LV, AV, M, II>, Contact<N, LV>)
}

type BF<N, LV, AV, M, II> = DBVTBroadPhase<N,
                                           LV,
                                           Body<N, LV, AV, M, II>,
                                           AABB<N, LV>,
                                           Dispatcher<N, LV, AV, M, II>,
                                           PairwiseDetector<N, LV, AV, M, II>>;

struct Dispatcher<N, LV, AV, M, II> {
    margin:  N,
    simplex: JohnsonSimplex<N, AnnotatedPoint<LV>>
}

impl<N: Clone + Zero, LV: Clone + Zero + Dim, AV, M, II>
Dispatcher<N, LV, AV, M, II> {
    pub fn new(margin: N) -> Dispatcher<N, LV, AV, M, II> {
        let template = RecursionTemplate::new::<AnnotatedPoint<LV>>();
        let simplex  = JohnsonSimplex::new(template, Zero::zero());
        Dispatcher {
            margin:  margin,
            simplex: simplex
        }
    }
}

impl<N: Clone, LV: Clone, AV, M, II>
     dispatcher::Dispatcher<Body<N, LV, AV, M, II>, PairwiseDetector<N, LV, AV, M, II>>
for Dispatcher<N, LV, AV, M, II> {
    fn dispatch(&self, a: &Body<N, LV, AV, M, II>, b: &Body<N, LV, AV, M, II>)
        -> PairwiseDetector<N, LV, AV, M, II> {
        match (*a, *b) {
            (RigidBody(rb1), RigidBody(rb2)) => {
                RB(DefaultDefault::new(rb1.geom(), rb2.geom(), &self.simplex, self.margin.clone()))
            },
            _ => Unsuported
        }
    }

    fn is_valid(&self,
                a: &Body<N, LV, AV, M, II>,
                b: &Body<N, LV, AV, M, II>)
                -> bool {
        if borrow::ref_eq(a, b) {
            return false
        }

        match (*a, *b) {
            (RigidBody(a), RigidBody(b)) => a.can_move() || b.can_move(),
            _ => true
        }
    }
}


pub struct DBVTBodiesBodies<N, LV, AV, M, II> {
    broad_phase: BF<N, LV, AV, M, II>,
}

impl<N:  'static + Clone + Zero + Ord + NumCast,
     LV: 'static + Zero + Dim + Bounded + ScalarAdd<N> + ScalarSub<N> + Neg<LV> +
         Ord + Orderable + Add<LV, LV> + ScalarDiv<N> + Clone + Sub<LV, LV> + Norm<N>,
     AV: 'static,
     M:  'static + Translation<LV> + Mul<M, M>,
     II: 'static>
DBVTBodiesBodies<N, LV, AV, M, II> {
    pub fn new(margin: N) -> DBVTBodiesBodies<N, LV, AV, M, II> {
        let dispatcher = Dispatcher::new(margin.clone());

        DBVTBodiesBodies {
            broad_phase: DBVTBroadPhase::new(dispatcher, margin)
        }
    }
}

impl<N:  'static + ApproxEq<N> + DivisionRing + Real + Float + Ord + Clone,
     LV: 'static + VectorSpace<N> + Dim + Dot<N> + Norm<N> + UniformSphereSample + ApproxEq<N> +
         SubDot<N> + ScalarAdd<N> + ScalarSub<N> + Cross<AV> + Ord + Orderable + Bounded + Basis +
         Eq + Clone,
     AV: 'static + ScalarMul<N> + Neg<AV>,
     M:  'static + Rotation<AV> + Rotate<LV> + Translation<LV> + Translatable<LV, M> +
         Transform<LV> + One + Mul<M, M> + Inv,
     II: 'static>
Detector<N, Body<N, LV, AV, M, II>, Constraint<N, LV, AV, M, II>>
for DBVTBodiesBodies<N, LV, AV, M, II> {
    fn add(&mut self, o: @mut Body<N, LV, AV, M, II>) {
        self.broad_phase.add(o);
    }

    fn remove(&mut self, o: @mut Body<N, LV, AV, M, II>) {
        self.broad_phase.remove(o);
    }

    fn update(&mut self) {
        self.broad_phase.update();

        let pairs = self.broad_phase.pairs_mut().elements_mut();

        for p in pairs.mut_iter() {
            match p.value {
                RB(ref mut d) => {
                    let rb1 = p.key.first.object.to_rigid_body_or_fail();
                    let rb2 = p.key.second.object.to_rigid_body_or_fail();

                    d.update(rb1.transform_ref(), rb1.geom(), rb2.transform_ref(), rb2.geom())
                },
                Unsuported => { }
            }
        }
    }

    fn interferences(&mut self, out: &mut ~[Constraint<N, LV, AV, M, II>]) {
        let mut collector = ~[];
        let pairs         = self.broad_phase.pairs_mut().elements_mut();

        for p in pairs.mut_iter() {
            match p.value {
                RB(ref mut d) => {
                    d.colls(&mut collector);

                    for c in collector.iter() {
                        out.push(RBRB(p.key.first.object, p.key.second.object, c.clone()))
                    }

                    collector.clear()
                },
                Unsuported => { }
            }
        }
    }

    fn activate(&mut self,
                body: @mut Body<N, LV, AV, M, II>,
                out:  &mut ~[Constraint<N, LV, AV, M, II>]) {
        let mut collector = ~[];
        let mut out_ids = ~[];

        self.broad_phase.activate(body, &mut out_ids);

        for i in out_ids.iter() {
            let e = &mut self.broad_phase.pairs_mut().elements_mut()[*i];

            match e.value {
                RB(ref mut d) => {
                    let rb1 = e.key.first.object.to_rigid_body_or_fail();
                    let rb2 = e.key.second.object.to_rigid_body_or_fail();

                    // FIXME: is the update needed? Or do we have enough guarantees to avoid it?
                    d.update(rb1.transform_ref(), rb1.geom(), rb2.transform_ref(), rb2.geom());

                    d.colls(&mut collector);

                    for c in collector.iter() {
                        out.push(RBRB(e.key.first.object, e.key.second.object, c.clone()))
                    }

                    collector.clear()
                },
                Unsuported => { }
            }
            
        }
    }

    fn deactivate(&mut self, body: @mut Body<N, LV, AV, M, II>) {
        self.broad_phase.deactivate(body)
    }
}
