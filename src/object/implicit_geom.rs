use std::num::One;
use nalgebra::mat::{Translation, Rotate, Transform};
use nalgebra::vec::AlgebraicVecExt;
use ncollide::bounding_volume::aabb::{HasAABB, AABB};
use ncollide::geom::implicit::{Implicit, HasMargin};
use ncollide::geom::ball;
use ncollide::geom::box;
use ncollide::geom::cone;
use ncollide::geom::cylinder;
use ncollide::geom::plane;
use ncollide::geom::compound::CompoundAABB;
use ncollide::ray::ray::{Ray, RayCast, RayCastWithTransform};

/// Enumeration grouping all common shapes. Used to simplify collision detection
/// dispatch.
pub enum DefaultGeom<N, V, M, II> { // FIXME: rename that
    Plane(plane::Plane<N, V>),
    Compound(@CompoundAABB<N, V, M, DefaultGeom<N, V, M, II>>),
    Implicit(ImplicitGeom<N, V, M>)
}

pub enum ImplicitGeom<N, V, M> {
    Ball(ball::Ball<N>),
    Box(box::Box<N, V>),
    Cone(cone::Cone<N>),
    Cylinder(cylinder::Cylinder<N>),
}

impl<N: One + ToStr, V: ToStr, M, II: ToStr> DefaultGeom<N, V, M, II> {
    #[inline]
    pub fn new_plane(p: plane::Plane<N, V>) -> DefaultGeom<N, V, M, II> {
        Plane(p)
    }

    #[inline]
    pub fn new_compound(c: @CompoundAABB<N, V, M, DefaultGeom<N, V, M, II>>)
        -> DefaultGeom<N, V, M, II> {
        Compound(c)
    }

    #[inline]
    pub fn new_ball(b: ball::Ball<N>) -> DefaultGeom<N, V, M, II> {
        Implicit(Ball(b))
    }

    #[inline]
    pub fn new_cylinder(b: cylinder::Cylinder<N>) -> DefaultGeom<N, V, M, II> {
        Implicit(Cylinder(b))
    }

    #[inline]
    pub fn new_box(b: box::Box<N, V>) -> DefaultGeom<N, V, M, II> {
        Implicit(Box(b))
    }

    #[inline]
    pub fn new_cone(b: cone::Cone<N>) -> DefaultGeom<N, V, M, II> {
        Implicit(Cone(b))
    }

}

impl<N, V, M, II> DefaultGeom<N, V, M, II> {
    /**
     * Convenience method to extract a plane from the enumation. Fails if the
     * pattern `Plane(_)` is not matched.
     */
    #[inline]
    pub fn plane<'r>(&'r self) -> &'r plane::Plane<N, V> {
        match *self {
            Plane(ref p) => p,
            _ => fail!("Unexpected geometry: this is not a plane.")
        }
    }

    /**
     * Convenience method to extract a compound geometry from the enumation. Fails if the
     * pattern `Compound(_)` is not matched.
     */
    #[inline]
    pub fn compound(&self) -> @CompoundAABB<N, V, M, DefaultGeom<N, V, M, II>> {
        match *self {
            Compound(c) => c,
            _ => fail!("Unexpected geometry: this is not a compound.")
        }
    }

    /**
     * Convenience method to extract an implicit geometry from the enumation. Fails if the
     * pattern `Implicit(_)` is not matched.
     */
    #[inline]
    pub fn implicit<'r>(&'r self) -> &'r ImplicitGeom<N, V, M> {
        match *self {
            Implicit(ref i) => i,
            _ => fail!("Unexpected geometry: this is not an implicit.")
        }
    }

    /**
     * Convenience method to extract a ball from the enumation. Fails if the
     * pattern `Implicit(Ball(_))` is not matched.
     */
    #[inline]
    pub fn ball<'r>(&'r self) -> &'r ball::Ball<N> {
        match *self {
            Implicit(Ball(ref b)) => b,
            _ => fail!("Unexpected geometry: this is not a ball.")
        }
    }

    /**
     * Convenience method to extract a cone from the enumation. Fails if the
     * pattern `Implicit(Cone(_))` is not matched.
     */
    #[inline]
    pub fn cone<'r>(&'r self) -> &'r cone::Cone<N> {
        match *self {
            Implicit(Cone(ref c)) => c,
            _ => fail!("Unexpected geometry: this is not a ball.")
        }
    }

    /**
     * Convenience method to extract a cylinder from the enumation. Fails if the
     * pattern `Implicit(Cylinder(_))` is not matched.
     */
    #[inline]
    pub fn cylinder<'r>(&'r self) -> &'r cylinder::Cylinder<N> {
        match *self {
            Implicit(Cylinder(ref c)) => c,
            _ => fail!("Unexpected geometry: this is not a ball.")
        }
    }

    /**
     * Convenience method to extract a box from the enumation. Fails if the
     * pattern `Implicit(Box(_))` is not matched.
     */
    #[inline]
    pub fn box<'r>(&'r self) -> &'r box::Box<N, V> {
        match *self {
            Implicit(Box(ref b)) => b,
            _ => fail!("Unexpected geometry: this is not a ball.")
        }
    }
}

impl<N: NumCast + Primitive + Orderable + Algebraic + Signed + Clone + ToStr,
     V: AlgebraicVecExt<N> + Clone + ToStr,
     M: Translation<V> + Rotate<V> + Transform<V> + Mul<M, M>,
     II>
HasAABB<N, V, M> for DefaultGeom<N, V, M, II> {
    #[inline]
    fn aabb(&self, m: &M) -> AABB<N, V> {
        match *self {
            Plane(ref p)    => p.aabb(m),
            Compound(ref c) => c.aabb(m),
            Implicit(ref i) => {
                match *i {
                    Ball(ref b)     => b.aabb(m),
                    Box(ref b)      => b.aabb(m),
                    Cone(ref c)     => c.aabb(m),
                    Cylinder(ref c) => c.aabb(m),
                }
            }
        }
    }
}

impl<N: Algebraic + Bounded + Orderable + Primitive + Float + Clone + ToStr,
     V: 'static + AlgebraicVecExt<N> + Clone + ToStr,
     M: Rotate<V> + Transform<V>,
     II>
RayCast<N, V> for DefaultGeom<N, V, M, II> {
    #[inline]
    fn toi_with_ray(&self, ray: &Ray<V>) -> Option<N> {
        match *self {
            Plane(ref p)    => p.toi_with_ray(ray),
            Compound(ref c) => c.toi_with_ray(ray),
            Implicit(ref i) => {
                match *i {
                    Ball(ref b)     => b.toi_with_ray(ray),
                    Box(ref b)      => b.toi_with_ray(ray),
                    Cone(ref c)     => c.toi_with_ray(ray),
                    Cylinder(ref c) => c.toi_with_ray(ray),
                }
            }
        }
    }
}

impl<N: Algebraic + Bounded + Orderable + Primitive + Float + Clone + ToStr,
     V: 'static + AlgebraicVecExt<N> + Clone + ToStr,
     M: Rotate<V> + Transform<V>,
     II>
RayCastWithTransform<N, V, M> for DefaultGeom<N, V, M, II>;

impl<N: Clone + Add<N, N>, V, M> HasMargin<N> for ImplicitGeom<N, V, M> {
    #[inline]
    fn margin(&self) -> N {
        match *self {
            Ball(ref b)     => b.margin(),
            Box(ref b)      => b.margin(),
            Cone(ref c)     => c.margin(),
            Cylinder(ref c) => c.margin()
        }
    }
}

impl<N: Algebraic + Signed + Orderable + Clone,
     V: AlgebraicVecExt<N> + Clone,
     M: Translation<V> + Rotate<V> + Transform<V>>
Implicit<N, V, M> for ImplicitGeom<N, V, M> {
    #[inline]
    fn support_point_without_margin(&self, transform: &M, dir: &V) -> V {
        match *self {
            Ball(ref b)     => b.support_point_without_margin(transform, dir),
            Box(ref b)      => b.support_point_without_margin(transform, dir),
            Cone(ref c)     => c.support_point_without_margin(transform, dir),
            Cylinder(ref c) => c.support_point_without_margin(transform, dir),
        }
    }

    #[inline]
    fn support_point(&self, transform: &M, dir: &V) -> V {
        match *self {
            Ball(ref b)     => b.support_point(transform, dir),
            Box(ref b)      => b.support_point(transform, dir),
            Cone(ref c)     => c.support_point(transform, dir),
            Cylinder(ref c) => c.support_point(transform, dir),
        }
    }
}

