use std::num::One;
use nalgebra::traits::basis::Basis;
use nalgebra::traits::cross::Cross;
use nalgebra::traits::dim::Dim;
use nalgebra::traits::division_ring::DivisionRing;
use nalgebra::traits::dot::Dot;
use nalgebra::traits::norm::Norm;
use nalgebra::traits::rotation::{Rotate, Rotation};
use nalgebra::traits::sample::UniformSphereSample;
use nalgebra::traits::scalar_op::ScalarMul;
use nalgebra::traits::sub_dot::SubDot;
use nalgebra::traits::transformation::Transform;
use nalgebra::traits::translation::{Translation, Translatable};
use nalgebra::traits::vector_space::VectorSpace;
use ncollide::geom::ball::Ball;
use ncollide::geom::minkowski_sum::AnnotatedPoint;
use ncollide::contact::Contact;
use ncollide::narrow::algorithm::johnson_simplex::JohnsonSimplex;
use ncollide::narrow::collision_detector::CollisionDetector;
use ncollide::narrow::implicit_implicit::ImplicitImplicit;
use ncollide::narrow::ball_ball::BallBall;
use OSCMG = ncollide::narrow::one_shot_contact_manifold_generator::OneShotContactManifoldGenerator;
use UTF = ncollide::narrow::one_shot_contact_manifold_generator::UnsafeTransformedRef;
use ncollide::narrow::plane_implicit::{PlaneImplicit, ImplicitPlane};
use object::implicit_geom::{DefaultGeom, Plane, Ball, Implicit};
use I = object::implicit_geom::DynamicImplicit;

type S<N, LV> = JohnsonSimplex<N, AnnotatedPoint<LV>>;

enum DefaultDefault<N, LV, AV, M, II> {
    BallBall        (BallBall<N, LV>),
    BallPlane       (ImplicitPlane<N, LV, Ball<N, LV>>),
    PlaneBall       (PlaneImplicit<N, LV, Ball<N, LV>>),
    BallImplicit    (ImplicitImplicit<S<N, LV>, Ball<N, LV>, ~I<N, LV, M, II>, N, LV>),
    ImplicitBall    (ImplicitImplicit<S<N, LV>, ~I<N, LV, M, II>, Ball<N, LV>, N, LV>),
    PlaneImplicit   (OSCMG<PlaneImplicit<N, LV, UTF<N, M, ~I<N, LV, M, II>>>, N, LV, AV, M>),
    ImplicitPlane   (OSCMG<ImplicitPlane<N, LV, UTF<N, M, ~I<N, LV, M, II>>>, N, LV, AV, M>),
    ImplicitImplicit(OSCMG<ImplicitImplicit<S<N, LV>, UTF<N, M, ~I<N, LV, M, II>>, ~I<N, LV, M, II>, N, LV>, N, LV, AV, M>)
}

impl<N: Clone, LV: Clone, AV, M, II> DefaultDefault<N, LV, AV, M, II> {
    pub fn new(g1:     &DefaultGeom<N, LV, M, II>,
               g2:     &DefaultGeom<N, LV, M, II>,
               s:      &JohnsonSimplex<N, AnnotatedPoint<LV>>,
               margin: N)
               -> DefaultDefault<N, LV, AV, M, II> {
        match (g1, g2) {
            (&Ball(_), &Ball(_))         => BallBall(BallBall::new()),
            (&Ball(_), &Plane(_))        => BallPlane(ImplicitPlane::new()),
            (&Plane(_), &Ball(_))        => PlaneBall(PlaneImplicit::new()),
            (&Implicit(_), &Ball(_))     => ImplicitBall(ImplicitImplicit::new(margin, s.clone())),
            (&Ball(_), &Implicit(_))     => BallImplicit(ImplicitImplicit::new(margin, s.clone())),
            (&Implicit(_), &Plane(_))    => ImplicitPlane(
                OSCMG::new(ImplicitPlane::new())
            ),
            (&Plane(_), &Implicit(_))    => PlaneImplicit(
                OSCMG::new(PlaneImplicit::new())
            ),
            (&Implicit(_), &Implicit(_)) => ImplicitImplicit(
                OSCMG::new(ImplicitImplicit::new(margin, s.clone()))
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
     LV: VectorSpace<N> + Dim + Dot<N> + Norm<N> + UniformSphereSample + ApproxEq<N> + SubDot<N> +
         Cross<AV> + Basis + Eq + Clone,
     AV: ScalarMul<N> + Neg<AV>,
     M:  Rotation<AV> + Rotate<LV> + Translation<LV> + Translatable<LV, M> + Transform<LV> + One,
     II>
CollisionDetector<N, LV, DefaultGeom<N, LV, M, II>, DefaultGeom<N, LV, M, II>>
for DefaultDefault<N, LV, AV, M, II>
 {
    #[inline]
    fn update(&mut self, g1: &DefaultGeom<N, LV, M, II>, g2: &DefaultGeom<N, LV, M, II>) {
        match *self {
            BallBall        (ref mut cd) => cd.update(g1.ball(),     g2.ball()),
            BallPlane       (ref mut cd) => cd.update(g1.ball(),     g2.plane()),
            PlaneBall       (ref mut cd) => cd.update(g2.ball(),     g1.plane()),
            BallImplicit    (ref mut cd) => cd.update(g1.ball(),     g2.implicit()),
            ImplicitBall    (ref mut cd) => cd.update(g1.implicit(), g2.ball()),
            PlaneImplicit   (ref mut cd) => cd.update(g2.implicit(), g1.plane()),
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
    fn colls(&mut self, out_colls: &mut ~[Contact<N, LV>]) {
        match *self {
            BallBall        (ref mut cd) => cd.colls(out_colls),
            BallPlane       (ref mut cd) => cd.colls(out_colls),
            PlaneBall       (ref mut cd) => {
                let begin = out_colls.len();
                cd.colls(out_colls);
                flip_colls(out_colls.mut_slice_from(begin));
            },
            BallImplicit    (ref mut cd) => cd.colls(out_colls),
            ImplicitBall    (ref mut cd) => cd.colls(out_colls),
            PlaneImplicit   (ref mut cd) => {
                let begin = out_colls.len();
                cd.colls(out_colls);
                flip_colls(out_colls.mut_slice_from(begin));
            }
            ImplicitPlane   (ref mut cd) => cd.colls(out_colls),
            ImplicitImplicit(ref mut cd) => cd.colls(out_colls)
        }
    }
}

fn flip_colls<N, LV: Neg<LV>>(colls: &mut [Contact<N, LV>]) {
    for c in colls.mut_iter() {
        c.flip();
    }
}
