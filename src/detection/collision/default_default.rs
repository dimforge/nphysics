use std::num::{Zero, One};
use nalgebra::traits::inv::Inv;
use nalgebra::traits::cross::Cross;
use nalgebra::traits::rotation::{Rotate, Rotation};
use nalgebra::traits::transformation::Transform;
use nalgebra::traits::translation::Translation;
use nalgebra::traits::vector::{Vec, AlgebraicVecExt};
use ncollide::geom::ball::Ball;
use ncollide::geom::minkowski_sum::AnnotatedPoint;
use ncollide::geom::compound::CompoundAABB;
use ncollide::contact::Contact;
use ncollide::broad::dispatcher;
use ncollide::narrow::algorithm::johnson_simplex::JohnsonSimplex;
use ncollide::narrow::collision_detector::CollisionDetector;
use ncollide::narrow::implicit_implicit;
use ncollide::narrow::implicit_implicit::ImplicitImplicit;
use ncollide::narrow::ball_ball;
use ncollide::narrow::ball_ball::BallBall;
use ncollide::narrow::compound_any::{AnyCompoundAABB, CompoundAABBAny};
use ncollide::narrow::compound_compound::CompoundAABBCompoundAABB;
use OSCMG = ncollide::narrow::one_shot_contact_manifold_generator::OneShotContactManifoldGenerator;
use ncollide::narrow::plane_implicit;
use ncollide::narrow::plane_implicit::{PlaneImplicit, ImplicitPlane};
use object::implicit_geom::{DefaultGeom, Plane, Ball, Implicit, Compound};
use object::implicit_geom::ImplicitGeom;

type S<N, LV> = JohnsonSimplex<N, AnnotatedPoint<LV>>;
type C<N, LV, M, II> = CompoundAABB<N, LV, M, DefaultGeom<N, LV, M, II>>;
type CA<N, LV, AV, M, II> = CompoundAABBAny<N, LV, M,
                                            DefaultGeom<N, LV, M, II>,
                                            Dispatcher<N, LV, AV, M, II>,
                                            DefaultDefault<N, LV, AV, M, II>>;
type AC<N, LV, AV, M, II> = AnyCompoundAABB<N, LV, M,
                                            DefaultGeom<N, LV, M, II>,
                                            Dispatcher<N, LV, AV, M, II>,
                                            DefaultDefault<N, LV, AV, M, II>>;

enum DefaultDefault<N, LV, AV, M, II> {
    BallBall(BallBall<N, LV, M>),
    BallPlane(ImplicitPlane<N, LV, M, Ball<N>>),
    PlaneBall(PlaneImplicit<N, LV, M, Ball<N>>),
    BallImplicit(ImplicitImplicit<S<N, LV>, Ball<N>, ImplicitGeom<N, LV, M>, N, LV>),
    ImplicitBall(ImplicitImplicit<S<N, LV>, ImplicitGeom<N, LV, M>, Ball<N>, N, LV>),
    PlaneImplicit(OSCMG<PlaneImplicit<N, LV, M, ImplicitGeom<N, LV, M>>, N, LV, AV, M>),
    ImplicitPlane(OSCMG<ImplicitPlane<N, LV, M, ImplicitGeom<N, LV, M>>, N, LV, AV, M>),
    ImplicitImplicit(OSCMG<ImplicitImplicit<S<N, LV>, ImplicitGeom<N, LV, M>, ImplicitGeom<N, LV, M>, N, LV>, N, LV, AV, M>),
    CompoundCompound(CompoundAABBCompoundAABB<N, LV, M,
                                              DefaultGeom<N, LV, M, II>,
                                              Dispatcher<N, LV, AV, M, II>,
                                              DefaultDefault<N, LV, AV, M, II>>),
    CompoundAny(CA<N, LV, AV, M, II>),
    AnyCompound(AC<N, LV, AV, M, II>)
}

impl<N: NumCast + Zero + Clone, LV: Clone, AV, M, II> DefaultDefault<N, LV, AV, M, II> {
    pub fn new(g1:     &DefaultGeom<N, LV, M, II>,
               g2:     &DefaultGeom<N, LV, M, II>,
               s:      &JohnsonSimplex<N, AnnotatedPoint<LV>>)
               -> DefaultDefault<N, LV, AV, M, II> {
        match (g1, g2) {
            (&Implicit(Ball(_)), &Implicit(Ball(_))) => {
                BallBall(BallBall::new(NumCast::from(0.1)))
            },
            (&Implicit(Ball(_)), &Plane(_))           => {
                BallPlane(ImplicitPlane::new(NumCast::from(0.1)))
            },
            (&Plane(_), &Implicit(Ball(_)))           => {
                PlaneBall(PlaneImplicit::new(NumCast::from(0.1)))
            },
            (&Implicit(_), &Implicit(Ball(_)))        => {
                ImplicitBall(ImplicitImplicit::new(NumCast::from(0.1), s.clone()))
            },
            (&Implicit(Ball(_)), &Implicit(_))        => {
                BallImplicit(ImplicitImplicit::new(NumCast::from(0.1), s.clone()))
            },
            (&Implicit(_), &Plane(_))       => {
                ImplicitPlane(OSCMG::new(NumCast::from(0.1), ImplicitPlane::new(Zero::zero())))
            },
            (&Plane(_), &Implicit(_))       => {
                PlaneImplicit(OSCMG::new(NumCast::from(0.1), PlaneImplicit::new(Zero::zero())))
            },
            (&Implicit(_), &Implicit(_))    => {
                ImplicitImplicit(
                OSCMG::new(NumCast::from(0.1), ImplicitImplicit::new(Zero::zero(), s.clone())))
            },
            (&Compound(c1), &Compound(c2))  => {
                CompoundCompound(CompoundAABBCompoundAABB::new(Dispatcher::new(s.clone()), c1, c2))
            },
            (&Compound(c), _)               => {
                CompoundAny(CompoundAABBAny::new(Dispatcher::new(s.clone()), c))
            },
            (_, &Compound(c))               => {
                AnyCompound(AnyCompoundAABB::new(Dispatcher::new(s.clone()), c))
            },
            _ => fail!("Dont know how to dispatch that.")
        }
    }
}

/**
 * Collision detector between two `DefaultGeometry`. Note that this is only a
 * wrapper on the collision detector specific to each geometry.
 */
impl<N: ApproxEq<N> + Num + Real + Float + Ord + Clone + ToStr + Algebraic,
     LV: 'static + AlgebraicVecExt<N> + Cross<AV> + ApproxEq<N> + Translation<LV> + Clone + ToStr +
         Rotate<LV> + Transform<LV>,
     AV: Vec<N> + ToStr,
     M:  Rotation<AV> + Rotate<LV> + Translation<LV> + Transform<LV> + Mul<M, M> + Inv + One,
     II>
CollisionDetector<N, LV, M, DefaultGeom<N, LV, M, II>, DefaultGeom<N, LV, M, II>>
for DefaultDefault<N, LV, AV, M, II> {
    #[inline]
    fn update(&mut self,
              m1: &M,
              g1: &DefaultGeom<N, LV, M, II>,
              m2: &M,
              g2: &DefaultGeom<N, LV, M, II>) {
        match *self {
            BallBall(ref mut cd)         => cd.update(m1, g1.ball(),     m2, g2.ball()),
            BallPlane(ref mut cd)        => cd.update(m1, g1.ball(),     m2, g2.plane()),
            PlaneBall(ref mut cd)        => cd.update(m1, g1.plane(),    m2, g2.ball()),
            BallImplicit(ref mut cd)     => cd.update(m1, g1.ball(),     m2, g2.implicit()),
            ImplicitBall(ref mut cd)     => cd.update(m1, g1.implicit(), m2, g2.ball()),
            PlaneImplicit(ref mut cd)    => cd.update(m1, g1.plane(),    m2, g2.implicit()),
            ImplicitPlane(ref mut cd)    => cd.update(m1, g1.implicit(), m2, g2.plane()),
            ImplicitImplicit(ref mut cd) => cd.update(m1, g1.implicit(), m2, g2.implicit()),
            CompoundCompound(ref mut cd) => cd.update(m1, g1.compound(), m2, g2.compound()),
            CompoundAny(ref mut cd)      => cd.update(m1, g1.compound(), m2, g2),
            AnyCompound(ref mut cd)      => cd.update(m1, g1,            m2, g2.compound())
        }
    }

    #[inline]
    fn num_coll(&self) -> uint {
        match *self {
            BallBall(ref cd)         => cd.num_coll(),
            BallPlane(ref cd)        => cd.num_coll(),
            PlaneBall(ref cd)        => cd.num_coll(),
            BallImplicit(ref cd)     => cd.num_coll(),
            ImplicitBall(ref cd)     => cd.num_coll(),
            PlaneImplicit(ref cd)    => cd.num_coll(),
            ImplicitPlane(ref cd)    => cd.num_coll(),
            ImplicitImplicit(ref cd) => cd.num_coll(),
            CompoundCompound(ref cd) => cd.num_coll(),
            CompoundAny(ref cd)      => cd.num_coll(),
            AnyCompound(ref cd)      => cd.num_coll(),
        }
    }

    #[inline]
    fn colls(&self, out_colls: &mut ~[Contact<N, LV>]) {
        match *self {
            BallBall(ref cd)         => cd.colls(out_colls),
            BallPlane(ref cd)        => cd.colls(out_colls),
            PlaneBall(ref cd)        => cd.colls(out_colls),
            BallImplicit(ref cd)     => cd.colls(out_colls),
            ImplicitBall(ref cd)     => cd.colls(out_colls),
            PlaneImplicit(ref cd)    => cd.colls(out_colls),
            ImplicitPlane(ref cd)    => cd.colls(out_colls),
            ImplicitImplicit(ref cd) => cd.colls(out_colls),
            CompoundCompound(ref cd) => cd.colls(out_colls),
            CompoundAny(ref cd)      => cd.colls(out_colls),
            AnyCompound(ref cd)      => cd.colls(out_colls)
        }
    }

    #[inline]
    fn toi(_:    Option<DefaultDefault<N, LV, AV, M, II>>,
           m1:   &M,
           dir:  &LV,
           dist: &N,
           g1:   &DefaultGeom<N, LV, M, II>,
           m2:   &M,
           g2:   &DefaultGeom<N, LV, M, II>)
           -> Option<N> {
        toi(m1, dir, dist, g1, m2, g2)
    }
}

#[inline]
pub fn toi<N:  ApproxEq<N> + Num + Real + Float + Ord + Clone + ToStr + Algebraic,
           LV: 'static + AlgebraicVecExt<N> + Cross<AV> + ApproxEq<N> + Translation<LV> + Clone +
               Rotate<LV> + Transform<LV> + ToStr,
           AV: Vec<N> + ToStr,
           M:  Rotation<AV> + Rotate<LV> + Translation<LV> + Transform<LV> + Mul<M, M> + Inv + One,
           II>(
           m1:   &M,
           dir:  &LV,
           dist: &N,
           g1:   &DefaultGeom<N, LV, M, II>,
           m2:   &M,
           g2:   &DefaultGeom<N, LV, M, II>)
           -> Option<N> {
    match (g1, g2) {
        (&Implicit(Ball(ref b1)), &Implicit(Ball(ref b2))) => ball_ball::toi(m1, dir, b1, m2, b2),
        (&Implicit(ref i), &Plane(ref p))      => plane_implicit::toi(m2, p, m1, dir, i),
        (&Plane(ref p), &Implicit(ref i))      => plane_implicit::toi(m1, p, m2, &-dir, i),
        (&Implicit(ref i1), &Implicit(ref i2)) => implicit_implicit::toi(m1, dir, i1, m2, i2),
        (&Compound(_), &Compound(_))   => fail!("Not yet implemented."), // CompoundCompound(),
        (&Compound(c), b) => {
            CollisionDetector::toi(None::<CA<N, LV, AV, M, II>>, m1, dir, dist, c, m2, b)
        },
        (a, &Compound(c)) => {
            CollisionDetector::toi(None::<AC<N, LV, AV, M, II>>, m1, dir, dist, a, m2, c)
        },
        _ => fail!("Cannot compute the toi of those two geometries.")
    }
}

struct Dispatcher<N, LV, AV, M, II> {
    simplex: JohnsonSimplex<N, AnnotatedPoint<LV>>
}

impl<N: Clone, LV: Clone, AV, M, II>
Dispatcher<N, LV, AV, M, II> {
    pub fn new(simplex: JohnsonSimplex<N, AnnotatedPoint<LV>>)
        -> Dispatcher<N, LV, AV, M, II> {
        Dispatcher {
            simplex: simplex
        }
    }
}

impl<N: NumCast + Zero + Clone, LV: Clone, AV, M, II>
     dispatcher::Dispatcher<DefaultGeom<N, LV, M, II>, DefaultDefault<N, LV, AV, M, II>>
for Dispatcher<N, LV, AV, M, II> {
    fn dispatch(&self, g1: &DefaultGeom<N, LV, M, II>, g2: &DefaultGeom<N, LV, M, II>)
                -> DefaultDefault<N, LV, AV, M, II> {
        DefaultDefault::new(g1, g2, &self.simplex)
    }

    fn is_valid(&self, _: &DefaultGeom<N, LV, M, II>, _: &DefaultGeom<N, LV, M, II>) -> bool {
        true
    }
}

impl<N: Clone, LV: Clone, AV, M, II> Clone for Dispatcher<N, LV, AV, M, II> {
    fn clone(&self) -> Dispatcher<N, LV, AV, M, II> {
        Dispatcher::new(self.simplex.clone())
    }
}
