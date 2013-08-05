use nalgebra::traits::vector_space::VectorSpace;
use nalgebra::traits::norm::Norm;
use nalgebra::traits::division_ring::DivisionRing;
use nalgebra::traits::dot::Dot;
use nalgebra::traits::sub_dot::SubDot;
use nalgebra::traits::dim::Dim;
use nalgebra::traits::sample::UniformSphereSample;
use ncollide::geom::ball::Ball;
use ncollide::geom::minkowski_sum::AnnotatedPoint;
use ncollide::contact::Contact;
use ncollide::narrow::algorithm::johnson_simplex::JohnsonSimplex;
use ncollide::narrow::collision_detector::CollisionDetector;
use ncollide::narrow::implicit_implicit::ImplicitImplicit;
use ncollide::narrow::ball_ball::BallBall;
use ICMG = ncollide::narrow::incremental_contact_manifold_generator::IncrementalContactManifoldGenerator;
use ncollide::narrow::plane_implicit::{PlaneImplicit, ImplicitPlane};
use object::implicit_geom::{DefaultGeom, Plane, Ball, Implicit};
use I = object::implicit_geom::DynamicImplicit;

type S<N, V> = JohnsonSimplex<N, AnnotatedPoint<V>>;

enum DefaultDefault<N, V, M, II> {
    BallBall        (BallBall<N, V>),
    BallPlane       (ImplicitPlane<N, V, Ball<N, V>>),
    PlaneBall       (PlaneImplicit<N, V, Ball<N, V>>),
    BallImplicit    (ImplicitImplicit<S<N, V>, Ball<N, V>, ~I<N, V, M, II>, N, V>),
    ImplicitBall    (ImplicitImplicit<S<N, V>, ~I<N, V, M, II>, Ball<N, V>, N, V>),
    PlaneImplicit   (ICMG<PlaneImplicit<N, V, ~I<N, V, M, II>>, N, V>),
    ImplicitPlane   (ICMG<ImplicitPlane<N, V, ~I<N, V, M, II>>, N, V>),
    ImplicitImplicit(ICMG<ImplicitImplicit<S<N, V>, ~I<N, V, M, II>, ~I<N, V, M, II>, N, V>, N, V>)
}

impl<N: Clone, V: Clone, M, II> DefaultDefault<N, V, M, II> {
    pub fn new(g1:     &DefaultGeom<N, V, M, II>,
               g2:     &DefaultGeom<N, V, M, II>,
               s:      &JohnsonSimplex<N, AnnotatedPoint<V>>,
               margin: N)
               -> DefaultDefault<N, V, M, II> {
        match (g1, g2) {
            (&Ball(_), &Ball(_))         => BallBall(BallBall::new()),
            (&Ball(_), &Plane(_))        => BallPlane(ImplicitPlane::new()),
            (&Plane(_), &Ball(_))        => PlaneBall(PlaneImplicit::new()),
            (&Implicit(_), &Ball(_))     => ImplicitBall(ImplicitImplicit::new(margin, s.clone())),
            (&Ball(_), &Implicit(_))     => BallImplicit(ImplicitImplicit::new(margin, s.clone())),
            (&Implicit(_), &Plane(_))    => ImplicitPlane(
                ICMG::new(ImplicitPlane::new())
            ),
            (&Plane(_), &Implicit(_))    => PlaneImplicit(
                ICMG::new(PlaneImplicit::new())
            ),
            (&Implicit(_), &Implicit(_)) => ImplicitImplicit(
                ICMG::new(ImplicitImplicit::new(margin, s.clone()))
            ),
            _ => fail!("Dont know how to dispatch that.")
        }
    }

}

/**
 * Collision detector between two `DefaultGeometry`. Note that this is only a
 * wrapper on the collision detector specific to each geometry.
 */
// FIXME: Ring + Real ?
impl<N: ApproxEq<N> + DivisionRing + Real + Float + Ord + Clone,
     V: VectorSpace<N> + Dim + Dot<N> + Norm<N> + UniformSphereSample + ApproxEq<N> + SubDot<N> +
        Eq + Clone,
     M,
     II>
CollisionDetector<N, V, DefaultGeom<N, V, M, II>, DefaultGeom<N, V, M, II>>
for DefaultDefault<N, V, M, II>
 {
    #[inline]
    fn update(&mut self, g1: &DefaultGeom<N, V, M, II>, g2: &DefaultGeom<N, V, M, II>) {
        match *self {
            BallBall        (ref mut cd) => cd.update(g1.ball(),     g2.ball()),
            BallPlane       (ref mut cd) => cd.update(g1.ball(),     g2.plane()),
            PlaneBall       (ref mut cd) => cd.update(g1.plane(),    g2.ball()),
            BallImplicit    (ref mut cd) => cd.update(g1.ball(),     g2.implicit()),
            ImplicitBall    (ref mut cd) => cd.update(g1.implicit(), g2.ball()),
            PlaneImplicit   (ref mut cd) => cd.update(g1.plane(),    g2.implicit()),
            ImplicitPlane   (ref mut cd) => cd.update(g1.implicit(), g2.plane()),
            ImplicitImplicit(ref mut cd) => cd.update(g1.implicit(), g2.implicit())
        }
    }

    #[inline]
    fn num_coll(&self) -> uint {
        match *self {
            BallBall        (ref cd) => cd.num_coll(),
            BallPlane       (ref cd) => cd.num_coll(),
            PlaneBall       (ref cd) => cd.num_coll(),
            BallImplicit    (ref cd) => cd.num_coll(),
            ImplicitBall    (ref cd) => cd.num_coll(),
            PlaneImplicit   (ref cd) => cd.num_coll(),
            ImplicitPlane   (ref cd) => cd.num_coll(),
            ImplicitImplicit(ref cd) => cd.num_coll()
        }
    }

    #[inline]
    fn colls(&mut self, out_colls: &mut ~[Contact<N, V>]) {
        match *self {
            BallBall        (ref mut cd) => cd.colls(out_colls),
            BallPlane       (ref mut cd) => cd.colls(out_colls),
            PlaneBall       (ref mut cd) => cd.colls(out_colls),
            BallImplicit    (ref mut cd) => cd.colls(out_colls),
            ImplicitBall    (ref mut cd) => cd.colls(out_colls),
            PlaneImplicit   (ref mut cd) => cd.colls(out_colls),
            ImplicitPlane   (ref mut cd) => cd.colls(out_colls),
            ImplicitImplicit(ref mut cd) => cd.colls(out_colls)
        }
    }
}
