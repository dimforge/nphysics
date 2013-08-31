use std::num::{Zero, One, Real, NumCast};
use nalgebra::vec::{VecExt, Dim, Iterable};
use nalgebra::mat::{Translation, Indexable};
use ncollide::geom::{Ball, Box, Cylinder, Cone, Plane};
use object::{DefaultGeom, Plane, Ball, Box, Cone, Cylinder, Compound, Implicit};

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
                let mut res: N = Zero::zero();

                for &(_, ref s) in c.shapes().iter() {
                    res = res + s.volume()
                }

                res
            },
            Implicit(ref i) => {
                match *i {
                    Ball(ref b)     => ball_volume(b.radius(), Dim::dim(None::<V>)),
                    Box(ref b)      => box_volume(&b.half_extents()),
                    Cone(ref c)     => cone_volume(&c.half_height(), &c.radius(), Dim::dim(None::<V>)),
                    Cylinder(ref c) => cylinder_volume(&c.half_height(), &c.radius(), Dim::dim(None::<V>)),
                }
            }
        }
    }

    #[inline]
    fn mass_properties(&self, density: &N) -> (N, V, II) {
        match *self {
            Plane(ref p) => p.mass_properties(density),
            Compound(c)  => {
                let mut mtot: N  = Zero::zero();
                let mut itot: II = Zero::zero();
                let mut ctot: V  = Zero::zero();

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
    let _pi: N = Real::pi();
    _pi * radius.pow(&NumCast::from(dim))
}

impl<N:  Real + Num + NumCast + Clone,
     V:  Zero + Dim,
     II: Zero + Indexable<(uint, uint), N>>
Volumetric<N, V, II> for Ball<N> {
    #[inline]
    fn volume(&self) -> N {
        ball_volume(self.radius(), Dim::dim(None::<V>))
    }

    fn mass_properties(&self, density: &N) -> (N, V, II) {
        let volume = ball_volume(self.radius(), Dim::dim(None::<V>));
        let mass   = volume * *density;

        let dim = Dim::dim(None::<V>);

        if dim == 2 {
            let diag = self.radius() * self.radius() * mass / NumCast::from(2.0);

            let mut res: II = Zero::zero();

            res.set((0, 0), diag);

            (mass, Zero::zero(), res)
        }
        else if dim == 3 {
            let _0: N  = Zero::zero();
            let diag: N = mass                              *
                          NumCast::from(2.0 / 5.0)          *
                          self.radius()                     *
                          self.radius();

            let mut res: II = Zero::zero();

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
        let dim  = Dim::dim(None::<V>);

        if dim == 2 {
            let _2: N   = NumCast::from(2.0);
            let _i12: N = NumCast::from(1.0 / 12.0);
            let w       = _i12 * mass * _2 * _2;
            let ix      = w * self.half_extents().at(0) * self.half_extents().at(0);
            let iy      = w * self.half_extents().at(1) * self.half_extents().at(1);

            let mut res: II = Zero::zero();

            res.set((0, 0), ix + iy);

            (mass, Zero::zero(), res)
        }
        else if dim == 3 {
            let _0: N   = Zero::zero();
            let _2: N   = NumCast::from(2.0);
            let _i12: N = NumCast::from(1.0 / 12.0);
            let w       = _i12 * mass * _2 * _2;
            let ix      = w * self.half_extents().at(0) * self.half_extents().at(0);
            let iy      = w * self.half_extents().at(1) * self.half_extents().at(1);
            let iz      = w * self.half_extents().at(2) * self.half_extents().at(2);

            let mut res: II = Zero::zero();

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
    let mut res: N = One::one();

    for half_extent in half_extents.iter() {
        res = res * *half_extent * NumCast::from(2.0)
    }

    res
}

impl<N:  Zero + One + NumCast + Num + Real + Clone,
     V:  Zero + Dim,
     II: Zero + Indexable<(uint, uint), N>>
Volumetric<N, V, II> for Cylinder<N> {
    fn volume(&self) -> N {
        let dim = Dim::dim(None::<V>);
        cylinder_volume(&self.half_height(), &self.radius(), dim)
    }

    fn mass_properties(&self, density: &N) -> (N, V, II) {
        let dim  = Dim::dim(None::<V>);
        let mass = cylinder_volume(&self.half_height(), &self.radius(), dim) * *density;

        if dim == 2 {
            // same as the box
            let _2:   N = NumCast::from(2.0f64);
            let _i12: N = NumCast::from(1.0f64 / 12.0);
            let w       = _i12 * mass * _2 * _2;
            let ix      = w * self.half_height() * self.half_height();
            let iy      = w * self.radius() * self.radius();

            let mut res: II = Zero::zero();

            res.set((0, 0), ix + iy);

            (mass, Zero::zero(), res)
        }
        else if dim == 3 {
            let sq_radius = self.radius() * self.radius();
            let sq_height = self.half_height() * self.half_height() * NumCast::from(4.0f64);
            let off_principal = mass * (sq_radius * NumCast::from(3.0) + sq_height) / NumCast::from(12.0);

            let mut res: II = Zero::zero();

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
     V:  Zero + Indexable<uint, N> + Dim,
     II: Zero + Indexable<(uint, uint), N>>
Volumetric<N, V, II> for Cone<N> {
    fn volume(&self) -> N {
        let dim = Dim::dim(None::<V>);

        cone_volume(&self.half_height(), &self.radius(), dim)
    }

    fn mass_properties(&self, density: &N) -> (N, V, II) {
        let dim  = Dim::dim(None::<V>);
        let mass = cone_volume(&self.half_height(), &self.radius(), dim) * *density;

        if dim == 2 {
            // FIXME: not sure about that…
            let mut res: II = Zero::zero();

            res.set(
                (0, 0),
                self.radius() * self.half_height() * self.half_height() * self.half_height()
                              / NumCast::from(3.0f64)
            );

            let mut center: V = Zero::zero();
            center.set(0, -self.half_height() / NumCast::from(2.0f64));

            (mass, center, res)
        }
        else if dim == 3 {
            let m_sq_radius = mass * self.radius() * self.radius();
            let m_sq_height = mass * self.half_height() * self.half_height() *
                                     NumCast::from(4.0f64);
            let off_principal = m_sq_radius * NumCast::from(3.0 / 20.0) +
                                m_sq_height * NumCast::from(3.0 / 5.0);

            let principal = m_sq_radius * NumCast::from(3.0 / 10.0);

            let mut res: II = Zero::zero();

            res.set((0, 0), principal);
            res.set((1, 1), off_principal.clone());
            res.set((2, 2), off_principal);

            let mut center: V = Zero::zero();
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
