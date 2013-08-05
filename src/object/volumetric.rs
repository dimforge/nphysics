use std::num::{Zero, One, Real, NumCast};
use nalgebra::traits::division_ring::DivisionRing;
use nalgebra::traits::iterable::Iterable;
use nalgebra::traits::indexable::Indexable;
use nalgebra::traits::dim::Dim;
use ncollide::geom::transformed::Transformed;
use ncollide::geom::ball::Ball;
use ncollide::geom::box::Box;
use ncollide::geom::cylinder::Cylinder;
use ncollide::geom::cone::Cone;
use ncollide::geom::plane::Plane;
use object::implicit_geom::{DefaultGeom, Plane, Ball, Implicit};

pub trait Volumetric<N, II> {
    fn volume(&self)      -> N;
    fn inertia(&self, &N) -> II;
}

impl<G: Volumetric<N, II>, M, N, II> Volumetric<N, II> for Transformed<G, M, N> {
    fn volume(&self) -> N {
        self.geom().volume()
    }

    fn inertia(&self, mass: &N) -> II {
        // FIXME: take in account the transform
        self.geom().inertia(mass)
    }
}

impl<N: Real + DivisionRing + NumCast + Clone,
     V: Clone + Iterable<N> + Dim,
     M,
     II: Zero + Indexable<(uint, uint), N>>
Volumetric<N, II> for DefaultGeom<N, V, M, II> {
    #[inline]
    fn volume(&self) -> N {
        match *self {
            // FIXME: dont know why the compiler is not happy calling volume() on Plane and Ball
            Plane(_)        => Zero::zero(), // p.volume(),
            Ball(ref b)     => ball_volume(b.radius(), Dim::dim::<V>()), // b.volume(),
            Implicit(ref i) => i.volume()
        }
    }

    #[inline]
    fn inertia(&self, mass: &N) -> II {
        match *self {
            Plane(ref p)    => p.inertia(mass),
            Ball(ref b)     => b.inertia(mass),
            Implicit(ref i) => i.inertia(mass)
        }
    }
}

fn ball_volume<N: Real + DivisionRing + NumCast>(radius: N, dim: uint) -> N {
    Real::pi::<N>() * radius.pow(&NumCast::from::<N, uint>(dim))
}

impl<N: Real + DivisionRing + NumCast + Clone,
     V: Clone + Dim,
     II: Zero + Indexable<(uint, uint), N>>
Volumetric<N, II> for Ball<N, V> {
    #[inline]
    fn volume(&self)  -> N {
        ball_volume(self.radius(), Dim::dim::<V>())
    }

    fn inertia(&self, mass: &N) -> II {
        let dim = Dim::dim::<V>();

        if dim == 2 {
            let diag = self.radius() * self.radius() * *mass / NumCast::from::<N, float>(2.0);

            let mut res = Zero::zero::<II>();

            res.set((0, 0), diag);

            res
        }
        else if dim == 3 {
            let _0   = Zero::zero::<N>();
            let diag = NumCast::from::<N, float>(2.0 / 5.0) *
                       *mass                                *
                       self.radius()                        *
                       self.radius();

            let mut res = Zero::zero::<II>();

            res.set((0, 0), diag.clone());
            res.set((1, 1), diag.clone());
            res.set((2, 2), diag.clone());

            res
        }
        else {
            fail!("Inertia tensor for n-dimensional boxes, n > 3, is not implemented.")
        }
    }
}

impl<N:  Zero + One + NumCast + DivisionRing + Clone,
     V:  Clone + Iterable<N> + Indexable<uint, N> + Dim,
     II: Zero + Indexable<(uint, uint), N>>
Volumetric<N, II> for Box<N, V> {
    fn volume(&self) -> N {
        let mut res  = One::one::<N>();

        let he = self.half_extents();

        for half_extent in he.iter() {
            res = res * *half_extent * NumCast::from::<N, float>(2.0)
        }

        res
    }

    fn inertia(&self, mass: &N) -> II {
        let dim = Dim::dim::<V>();

        if dim == 2 {
            let _2   = NumCast::from::<N, float>(2.0);
            let _i12 = NumCast::from::<N, float>(1.0 / 12.0);
            let w    = _i12 * *mass * _2 * _2;
            let ix   = w * self.half_extents().at(0) * self.half_extents().at(0);
            let iy   = w * self.half_extents().at(1) * self.half_extents().at(1);

            let mut res = Zero::zero::<II>();

            res.set((0, 0), ix + iy);

            res
        }
        else if dim == 3 {
            let _0   = Zero::zero::<N>();
            let _2   = NumCast::from::<N, float>(2.0);
            let _i12 = NumCast::from::<N, float>(1.0 / 12.0);
            let w    = _i12 * *mass * _2 * _2;
            let ix   = w * self.half_extents().at(0) * self.half_extents().at(0);
            let iy   = w * self.half_extents().at(1) * self.half_extents().at(1);
            let iz   = w * self.half_extents().at(2) * self.half_extents().at(2);

            let mut res  = Zero::zero::<II>();

            res.set((0, 0), iy + iz);
            res.set((1, 1), ix + iz);
            res.set((2, 2), ix + iy);

            res
        }
        else {
            fail!("Inertia tensor for n-dimensional boxes, n > 3, is not implemented.")
        }
    }
}

impl<N:  Zero + One + NumCast + DivisionRing + Real + Clone,
     II: Zero + Dim + Indexable<(uint, uint), N>>
Volumetric<N, II> for Cylinder<N> {
    fn volume(&self) -> N {
        let dim = Dim::dim::<II>();

        if dim == 2 {
            // same as a rectangle
            self.half_height() * self.radius() * NumCast::from(4.0f64)
        }
        else if dim == 3 {
            self.half_height() * self.radius() * self.radius() * Real::pi() * NumCast::from(2.0f64)
        }
        else {
            fail!("Volume for n-dimensional cylinders, n > 3, is not implemented.")
        }
    }

    fn inertia(&self, mass: &N) -> II {
        let dim = Dim::dim::<II>();

        if dim == 2 {
            // same as the box
            let _2:   N = NumCast::from(2.0f64);
            let _i12: N = NumCast::from(1.0f64 / 12.0);
            let w       = _i12 * *mass * _2 * _2;
            let ix      = w * self.half_height() * self.half_height();
            let iy      = w * self.radius() * self.radius();

            let mut res = Zero::zero::<II>();

            res.set((0, 0), ix + iy);

            res
        }
        else if dim == 3 {
            let sq_radius = self.radius() * self.radius();
            let sq_height = self.half_height() * self.half_height() * NumCast::from(4.0f64);
            let off_principal = mass * (NumCast::from::<N, f64>(3.0) * sq_radius + sq_height)
                                / NumCast::from::<N, f64>(12.0);

            let mut res = Zero::zero::<II>();

            res.set((0, 0), mass * sq_radius / NumCast::from(2.0f64));
            res.set((1, 1), off_principal.clone());
            res.set((2, 2), off_principal);

            res
        }
        else {
            fail!("Inertia tensor for n-dimensional cylinder, n > 3, is not implemented.")
        }
    }
}

impl<N:  Zero + One + NumCast + DivisionRing + Real + Clone,
     II: Zero + Dim + Indexable<(uint, uint), N>>
Volumetric<N, II> for Cone<N> {
    fn volume(&self) -> N {
        let dim = Dim::dim::<II>();

        if dim == 2 {
            // same as a isosceles triangle
            self.radius() * self.half_height() * NumCast::from(2.0f64)
        }
        else if dim == 3 {
            self.radius() * self.radius()      *
            Real::pi()    * self.half_height() *
            NumCast::from(2.0f64 / 3.0)
        }
        else {
            fail!("Volume for n-dimensional cone, n > 3, is not implemented.")
        }
    }

    fn inertia(&self, mass: &N) -> II {
        let dim = Dim::dim::<II>();

        if dim == 2 {
            // FIXME: not sure about that…
            let mut res = Zero::zero::<II>();

            res.set(
                (0, 0),
                self.radius() * self.half_height() * self.half_height() * self.half_height()
                              / NumCast::from(3.0f64)
            );

            res
        }
        else if dim == 3 {
            let m_sq_radius = mass * self.radius() * self.radius();
            let m_sq_height = mass * self.half_height() * self.half_height() *
                                     NumCast::from(4.0f64);
            let off_principal = NumCast::from::<N, f64>(3.0 / 20.0) * m_sq_radius +
                                NumCast::from::<N, f64>(3.0 / 5.0)  * m_sq_height;

            let principal = NumCast::from::<N, f64>(3.0 / 10.0) * m_sq_radius;

            let mut res = Zero::zero::<II>();

            res.set((0, 0), principal);
            res.set((1, 1), off_principal.clone());
            res.set((2, 2), off_principal);

            res
        }

        else {
            fail!("Inertia tensor for n-dimensional cone, n > 3, is not implemented.")
        }
    }
}

impl<N: Zero, V, II: Zero> Volumetric<N, II> for Plane<V> {
    #[inline]
    fn volume(&self) -> N {
        Zero::zero()
    }

    #[inline]
    fn inertia(&self, _: &N) -> II {
        Zero::zero()
    }
}
