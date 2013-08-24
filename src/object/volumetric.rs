use std::num::{Zero, One, Real, NumCast};
use nalgebra::traits::iterable::Iterable;
use nalgebra::traits::indexable::Indexable;
use nalgebra::traits::dim::Dim;
use nalgebra::traits::translation::Translation;
use nalgebra::traits::vector::VecExt;
use ncollide::geom::ball::Ball;
use ncollide::geom::box::Box;
use ncollide::geom::cylinder::Cylinder;
use ncollide::geom::cone::Cone;
use ncollide::geom::plane::Plane;
use object::implicit_geom::{DefaultGeom, Plane, Ball, Box, Cone, Cylinder, Compound, Implicit};

pub trait InertiaTensor<N, V, M> {
    fn to_world_space(&self, &M) -> Self;
    fn to_relative_wrt_point(&self, &N, &V) -> Self;
}

pub trait Volumetric<N, V, II> {
    fn volume(&self) -> N;
    fn mass_properties(&self, &N) -> (N, V, II);
}

impl<N: Real + Num + NumCast + Clone + ToStr,
     V: Clone + VecExt<N> + ToStr,
     M: Translation<V>,
     II: Zero + Indexable<(uint, uint), N> + Add<II, II> + InertiaTensor<N, V, M> + Dim + ToStr>
Volumetric<N, V, II> for DefaultGeom<N, V, M, II> {
    #[inline]
    fn volume(&self) -> N {
        match *self {
            Plane(_)        => Zero::zero(),
            Compound(c)     => {
                let mut res = Zero::zero::<N>();

                for &(_, ref s) in c.shapes().iter() {
                    res = res + s.volume()
                }

                res
            },
            Implicit(ref i) => {
                match *i {
                    Ball(ref b)     => ball_volume(b.radius(), Dim::dim::<V>().max(&2)),
                    Box(ref b)      => box_volume(&b.half_extents()),
                    Cone(ref c)     => cone_volume(&c.half_height(), &c.radius(), Dim::dim::<V>().max(&2)),
                    Cylinder(ref c) => cylinder_volume(&c.half_height(), &c.radius(), Dim::dim::<V>().max(&2)),
                }
            }
        }
    }

    #[inline]
    fn mass_properties(&self, density: &N) -> (N, V, II) {
        match *self {
            Plane(ref p) => p.mass_properties(density),
            Compound(c)  => {
                let mut mtot = Zero::zero::<N>();
                let mut itot = Zero::zero::<II>();
                let mut ctot = Zero::zero::<V>();

                for &(ref m, ref s) in c.shapes().iter() {
                    let (mpart, cpart, ipart) = s.mass_properties(density);
                    mtot = mtot + mpart;
                    itot = itot + ipart.to_world_space(m).to_relative_wrt_point(&mpart, &m.translation());
                    ctot = ctot + cpart * mpart;
                }

                ctot = ctot / mtot;

                (mtot, ctot, itot)
            },
            Implicit(ref i) => {
                match *i {
                    Ball(ref b)     => b.mass_properties(density),
                    Box(ref b)      => b.mass_properties(density),
                    Cone(ref c)     => c.mass_properties(density),
                    Cylinder(ref c) => c.mass_properties(density),
                }
            }
        }
    }
}

#[inline]
fn ball_volume<N: Real + Num + NumCast>(radius: N, dim: uint) -> N {
    Real::pi::<N>() * radius.pow(&NumCast::from::<N, uint>(dim))
}

impl<N:  Real + Num + NumCast + Clone,
     V:  Zero,
     II: Zero + Indexable<(uint, uint), N> + Dim>
Volumetric<N, V, II> for Ball<N> {
    #[inline]
    fn volume(&self) -> N {
        ball_volume(self.radius(), Dim::dim::<II>().max(&2))
    }

    fn mass_properties(&self, density: &N) -> (N, V, II) {
        let volume = ball_volume(self.radius(), Dim::dim::<II>().max(&2));
        let mass   = volume * *density;

        let dim = Dim::dim::<II>().max(&2);

        if dim == 2 {
            let diag = self.radius() * self.radius() * mass / NumCast::from::<N, float>(2.0);

            let mut res = Zero::zero::<II>();

            res.set((0, 0), diag);

            (mass, Zero::zero(), res)
        }
        else if dim == 3 {
            let _0   = Zero::zero::<N>();
            let diag = NumCast::from::<N, float>(2.0 / 5.0) *
                       mass                                 *
                       self.radius()                        *
                       self.radius();

            let mut res = Zero::zero::<II>();

            res.set((0, 0), diag.clone());
            res.set((1, 1), diag.clone());
            res.set((2, 2), diag.clone());

            (mass, Zero::zero(), res)
        }
        else {
            fail!("Inertia tensor for n-dimensional balls, n > 3, is not implemented.")
        }
    }
}

impl<N:  Zero + One + NumCast + Num + Clone + ToStr,
     V:  Clone + VecExt<N> + ToStr,
     II: Zero + Indexable<(uint, uint), N> + ToStr>
Volumetric<N, V, II> for Box<N, V> {
    fn volume(&self) -> N {
        box_volume(&self.half_extents())
    }

    fn mass_properties(&self, density: &N) -> (N, V, II) {
        let mass = box_volume(&self.half_extents()) * *density;
        let dim  = Dim::dim::<V>();

        if dim == 2 {
            let _2   = NumCast::from::<N, float>(2.0);
            let _i12 = NumCast::from::<N, float>(1.0 / 12.0);
            let w    = _i12 * mass * _2 * _2;
            let ix   = w * self.half_extents().at(0) * self.half_extents().at(0);
            let iy   = w * self.half_extents().at(1) * self.half_extents().at(1);

            let mut res = Zero::zero::<II>();

            res.set((0, 0), ix + iy);

            (mass, Zero::zero(), res)
        }
        else if dim == 3 {
            let _0   = Zero::zero::<N>();
            let _2   = NumCast::from::<N, float>(2.0);
            let _i12 = NumCast::from::<N, float>(1.0 / 12.0);
            let w    = _i12 * mass * _2 * _2;
            let ix   = w * self.half_extents().at(0) * self.half_extents().at(0);
            let iy   = w * self.half_extents().at(1) * self.half_extents().at(1);
            let iz   = w * self.half_extents().at(2) * self.half_extents().at(2);

            let mut res  = Zero::zero::<II>();

            res.set((0, 0), iy + iz);
            res.set((1, 1), ix + iz);
            res.set((2, 2), ix + iy);

            (mass, Zero::zero(), res)
        }
        else {
            fail!("Inertia tensor for n-dimensional boxes, n > 3, is not implemented.")
        }
    }
}

#[inline]
fn box_volume<N:  Zero + One + NumCast + Num + Clone,
              V:  Clone + VecExt<N>>(
              half_extents: &V)
              -> N {
    let mut res  = One::one::<N>();

    for half_extent in half_extents.iter() {
        res = res * *half_extent * NumCast::from::<N, float>(2.0)
    }

    res
}

impl<N:  Zero + One + NumCast + Num + Real + Clone,
     V:  Zero,
     II: Zero + Dim + Indexable<(uint, uint), N>>
Volumetric<N, V, II> for Cylinder<N> {
    fn volume(&self) -> N {
        let dim = Dim::dim::<II>().max(&2);
        cylinder_volume(&self.half_height(), &self.radius(), dim)
    }

    fn mass_properties(&self, density: &N) -> (N, V, II) {
        let dim  = Dim::dim::<II>().max(&2);
        let mass = cylinder_volume(&self.half_height(), &self.radius(), dim) * *density;

        if dim == 2 {
            // same as the box
            let _2:   N = NumCast::from(2.0f64);
            let _i12: N = NumCast::from(1.0f64 / 12.0);
            let w       = _i12 * mass * _2 * _2;
            let ix      = w * self.half_height() * self.half_height();
            let iy      = w * self.radius() * self.radius();

            let mut res = Zero::zero::<II>();

            res.set((0, 0), ix + iy);

            (mass, Zero::zero(), res)
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

            (mass, Zero::zero(), res)
        }
        else {
            fail!("Inertia tensor for n-dimensional cylinder, n > 3, is not implemented.")
        }
    }
}

#[inline]
fn cylinder_volume<N: Zero + One + NumCast + Num + Real + Clone>(
                   half_height: &N,
                   radius:      &N,
                   dim:         uint)
                   -> N {
    if dim == 2 {
        // same as a rectangle
        half_height * *radius * NumCast::from(4.0f64)
    }
    else if dim == 3 {
        half_height * *radius * *radius * Real::pi() * NumCast::from(2.0f64)
    }
    else {
        fail!("Volume for n-dimensional cylinders, n > 3, is not implemented.")
    }
}

impl<N:  Zero + One + NumCast + Num + Real + Clone,
     V:  Zero + Indexable<uint, N>,
     II: Zero + Dim + Indexable<(uint, uint), N>>
Volumetric<N, V, II> for Cone<N> {
    fn volume(&self) -> N {
        let dim = Dim::dim::<II>().max(&2);

        cone_volume(&self.half_height(), &self.radius(), dim)
    }

    fn mass_properties(&self, density: &N) -> (N, V, II) {
        let dim  = Dim::dim::<II>().max(&2);
        let mass = cone_volume(&self.half_height(), &self.radius(), dim) * *density;

        if dim == 2 {
            // FIXME: not sure about that…
            let mut res = Zero::zero::<II>();

            res.set(
                (0, 0),
                self.radius() * self.half_height() * self.half_height() * self.half_height()
                              / NumCast::from(3.0f64)
            );

            let mut center = Zero::zero::<V>();
            center.set(0, -self.half_height() / NumCast::from(2.0f64));

            (mass, center, res)
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

            let mut center = Zero::zero::<V>();
            center.set(0, -self.half_height() / NumCast::from(2.0f64));

            (mass, center, res)
        }

        else {
            fail!("Inertia tensor for n-dimensional cone, n > 3, is not implemented.")
        }
    }
}

#[inline]
fn cone_volume<N:  Zero + One + NumCast + Num + Real + Clone>(
               half_height: &N,
               radius:      &N,
               dim:         uint)
               -> N {
    if dim == 2 {
        // same as a isosceles triangle
        *radius * *half_height * NumCast::from(2.0f64)
    }
    else if dim == 3 {
        *radius * *radius * Real::pi() * *half_height * NumCast::from(2.0f64 / 3.0)
    }
    else {
        fail!("Volume for n-dimensional cone, n > 3, is not implemented.")
    }
}

impl<N: Zero, V: Zero, II: Zero> Volumetric<N, V, II> for Plane<N, V> {
    #[inline]
    fn volume(&self) -> N {
        Zero::zero()
    }

    #[inline]
    fn mass_properties(&self, _: &N) -> (N, V, II) {
        (Zero::zero(), Zero::zero(), Zero::zero())
    }
}
