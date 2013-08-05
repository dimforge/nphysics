use std::num::One;
use nalgebra::traits::scalar_op::{ScalarAdd, ScalarSub};
use nalgebra::traits::translation::Translation;
use nalgebra::traits::rotation::Rotate;
use nalgebra::traits::transformation::{Transform, Transformable, Transformation};
use ncollide::bounding_volume::aabb::{HasAABB, AABB};
use ncollide::geom::implicit::Implicit;
use ncollide::geom::ball;
use ncollide::geom::plane;
use object::volumetric::Volumetric;

/// Enumeration grouping all common shapes. Used to simplify collision detection
/// dispatch.
// FIXME: #[deriving(Clone)]
pub enum DefaultGeom<N, V, M, II> { // FIXME: rename that
    Plane(plane::Plane<V>),
    Ball(ball::Ball<N, V>),
    Implicit(~DynamicImplicit<N, V, M, II>)
}

impl<N, V, M: One + Transform<V> + Rotate<V>, II> DefaultGeom<N, V, M, II> {
    pub fn new_plane<G: Transformable<M, plane::Plane<V>>>(geom: &G) -> DefaultGeom<N, V, M, II> {
        Plane(geom.transformed(&One::one()))
    }
}

impl<N, V: Clone + Add<V, V> + Neg<V>, M: One + Translation<V> + Transform<V>, II>
DefaultGeom<N, V, M, II> {
    pub fn new_ball<G: Transformable<M, ball::Ball<N, V>>>(geom: &G) -> DefaultGeom<N, V, M, II> {
        Ball(geom.transformed(&One::one()))
    }
}

impl<N, V, M: One, II> DefaultGeom<N, V, M, II> {
    pub fn new_implicit<G:  Transformable<M, G2>,
                        G2: Send + DynamicImplicit<N, V, M, II>>(
                        geom: &G)
                        -> DefaultGeom<N, V, M, II> {
            Implicit(
                ~geom.transformed(&One::one())
                as ~DynamicImplicit<N, V, M, II>
                )
        }
}

impl<N, V, M, II> DefaultGeom<N, V, M, II> {
    /**
     * Convenience method to extract a ball from the enumation. Fails if the
     * pattern `Ball` is not matched.
     */
    #[inline]
    pub fn ball<'r>(&'r self) -> &'r ball::Ball<N, V> {
        match *self {
            Ball(ref b) => b,
            _ => fail!("Unexpected geometry: this is not a ball.")
        }
    }

    /**
     * Mutable version of `ball`.
     */
    #[inline]
    pub fn ball_mut<'r>(&'r mut self) -> &'r mut ball::Ball<N, V> {
        match *self {
            Ball(ref mut b) => b,
            _ => fail!("Unexpected geometry: this is not a ball.")
        }
    }

    /**
     * Convenience method to extract a plane from the enumation. Fails if the
     * pattern `Plane` is not matched.
     */
    #[inline]
    pub fn plane<'r>(&'r self) -> &'r plane::Plane<V> {
        match *self {
            Plane(ref p) => p,
            _ => fail!("Unexpected geometry: this is not a plane.")
        }
    }

    /**
     * Mutable version of `plane`.
     */
    #[inline]
    pub fn plane_mut<'r>(&'r mut self) -> &'r mut plane::Plane<V> {
        match *self {
            Plane(ref mut p) => p,
            _ => fail!("Unexpected geometry: this is not a plane.")
        }
    }

    #[inline]
    pub fn implicit<'r>(&'r self) -> &'r ~DynamicImplicit<N, V, M, II> {
        match *self {
            Implicit(ref i) => i,
            _ => fail!("Unexpected geometry: this is not an implicit.")
        }
    }

    /**
     * Mutable version of `implicit`.
     */
    #[inline]
    pub fn implicit_mut<'r>(&'r mut self)
                            -> &'r mut ~DynamicImplicit<N, V, M, II> {
        match *self {
            Implicit(ref mut i) => i,
            _ => fail!("Unexpected geometry: this is not an implicit.")
        }
    }
}

impl<N,
     V: Bounded + Neg<V> + ScalarAdd<N> + ScalarSub<N> + Ord + Clone,
     M,
     II>
DefaultGeom<N, V, M, II> {
    fn aabb(&self) -> AABB<V> {
        match *self {
            Plane(ref p)    => p.aabb(),
            Ball(ref b)     => b.aabb(),
            Implicit(ref i) => i.aabb()
        }
    }
}

impl<N,
     V: Clone + Add<V, V> + Neg<V>,
     M: One + Translation<V> + Transform<V> + Rotate<V>,
     II>
Transformation<M> for DefaultGeom<N, V, M, II> {
    fn transformation(&self) -> M {
        match *self {
            Plane(ref p)    => p.transformation(),
            Ball(ref b)     => b.transformation(),
            Implicit(ref i) => i.transformation(),
        }
    }

    fn inv_transformation(&self) -> M {
        match *self {
            Plane(ref p)    => p.inv_transformation(),
            Ball(ref b)     => b.inv_transformation(),
            Implicit(ref i) => i.inv_transformation(),
        }
    }


    fn transform_by(&mut self, m: &M) {
        match *self {
            Plane(ref mut p)    => p.transform_by(m),
            Ball(ref mut b)     => b.transform_by(m),
            Implicit(ref mut i) => i.transform_by(m),
        }
    }
}

pub trait DynamicImplicit<N, V, M, II>
: Implicit<V>       +
  Volumetric<N, II> +
  Transformation<M> +
  Transform<V>      +
  Translation<V>    +
  HasAABB<V> {
    // FIXME: those methods are workarounds: why dont trait objects of this
    // traits dont inherit from all the parent traits?
    fn _support_point(&self, dir: &V) -> V;
    fn _volume(&self)                 -> N;
    fn _inertia(&self, &N)            -> II;
    fn _transformation(&self)         -> M;
    fn _inv_transformation(&self)     -> M;
    fn _transform_by(&mut self, &M);
    fn _aabb(&self)                   -> AABB<V>;
    fn _transform_vec(&self, &V)      -> V;
    fn _inv_transform(&self, &V)      -> V;
    fn _translation(&self)            -> V;
    fn _inv_translation(&self)        -> V;
    fn _translate_by(&mut self, &V);
}

impl<T: Implicit<V> + Volumetric<N, II> + Transformation<M> + Transform<V> + Translation<V> + HasAABB<V>,
     V,
     N,
     M,
     II>
DynamicImplicit<N, V, M, II> for T {
    #[inline]
    fn _support_point(&self, dir: &V) -> V {
        self.support_point(dir)
    }

    #[inline]
    fn _volume(&self) -> N {
        self.volume()
    }

    #[inline]
    fn _inertia(&self, mass: &N) -> II {
        self.inertia(mass)
    }

    #[inline]
    fn _transformation(&self) -> M {
        self.transformation()
    }

    #[inline]
    fn _inv_transformation(&self) -> M {
        self.inv_transformation()
    }

    #[inline]
    fn _transform_by(&mut self, m: &M) {
        self.transform_by(m)
    }

    #[inline]
    fn _aabb(&self) -> AABB<V> {
        self.aabb()
    }

    #[inline]
    fn _transform_vec(&self, v: &V) -> V {
        self.transform_vec(v)
    }

    #[inline]
    fn _inv_transform(&self, v: &V) -> V {
        self.inv_transform(v)
    }

    #[inline]
    fn _translation(&self) -> V {
        self.translation()
    }

    #[inline]
    fn _inv_translation(&self) -> V {
        self.inv_translation()
    }


    #[inline]
    fn _translate_by(&mut self, t: &V) {
        self.translate_by(t)
    }
}

// FIXME: all the following are workarounds to make
// ~ImplicitVolumetricTransformationBoundingVolume implement all the traits it
// inherits from. This is a compiler issue.
impl<N, V, M, II> Implicit<V> for ~DynamicImplicit<N, V, M, II> {
    #[inline]
    fn support_point(&self, dir: &V) -> V {
        self._support_point(dir)
    }
}

impl<N, V, M, II> Volumetric<N, II> for ~DynamicImplicit<N, V, M, II> {
    #[inline]
    fn volume(&self) -> N {
        self._volume()
    }

    #[inline]
    fn inertia(&self, mass: &N) -> II {
        self._inertia(mass)
    }
}

impl<N, V, M, II> Transformation<M> for ~DynamicImplicit<N, V, M, II> {
    #[inline]
    fn transformation(&self) -> M {
        self._transformation()
    }

    #[inline]
    fn inv_transformation(&self) -> M {
        self._inv_transformation()
    }


    #[inline]
    fn transform_by(&mut self, m: &M) {
        self._transform_by(m)
    }
}

impl<N, V, M, II> Transform<V> for ~DynamicImplicit<N, V, M, II> {
    fn transform_vec(&self, v: &V) -> V {
        self._transform_vec(v)
    }

    fn inv_transform(&self, v: &V) -> V {
        self._inv_transform(v)
    }
}

impl<N, V, M, II> Translation<V> for ~DynamicImplicit<N, V, M, II> {
    #[inline]
    fn translation(&self) -> V {
        self._translation()
    }

    #[inline]
    fn inv_translation(&self) -> V {
        self._inv_translation()
    }


    #[inline]
    fn translate_by(&mut self, t: &V) {
        self._translate_by(t)
    }
}

impl<N, V, M, II> HasAABB<V> for ~DynamicImplicit<N, V, M, II> {
    fn aabb(&self) -> AABB<V> {
        self._aabb()
    }
}
