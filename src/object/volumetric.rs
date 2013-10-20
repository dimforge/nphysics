use std::num::{Zero, One, Real};
use nalgebra::na::{Cast, VecExt, Dim, Iterable, Translation, Indexable};
use nalgebra::na;
use ncollide::geom::{Ball, Box, Cylinder, Capsule, Cone, Plane};
use ncollide::geom::{Geom, PlaneGeom, BallGeom, BoxGeom, ConeGeom, CylinderGeom, CapsuleGeom,
                     CompoundGeom, ImplicitGeom};

pub trait InertiaTensor<N, V, AV, M> {
    fn apply(&self, a: &AV) -> AV;
    fn to_world_space(&self, &M) -> Self;
    fn to_relative_wrt_point(&self, &N, &V) -> Self;
}

pub trait Volumetric<N, V, II> {
    // fn volume(&self) -> N;
    fn mass_properties(&self, &N) -> (N, V, II);
}

impl<N:  Send + Freeze + Real + Num + Cast<f32> + Clone,
     V:  Send + Freeze + Clone + VecExt<N>,
     AV,
     M:  Send + Freeze + Translation<V>,
     II: Zero + Indexable<(uint, uint), N> + Add<II, II> + InertiaTensor<N, V, AV, M> + Dim>
Volumetric<N, V, II> for Geom<N, V, M> {
    /*
    #[inline]
    fn volume(&self) -> N {
        match *self {
            PlaneGeom(_)        => na::zero(),
            CompoundGeom(c)     => {
                let mut res: N = na::zero();

                for &(_, ref s) in c.shapes().iter() {
                    res = res + s.volume()
                }

                res
            },
            ImplicitGeom(ref i) => {
                match *i {
                    BallGeom(ref b)     => ball_volume(&b.radius(), na::dim::<V>()),
                    BoxGeom(ref b)      => box_volume(&b.half_extents()),
                    ConeGeom(ref c)     => cone_volume(&c.half_height(), &c.radius(), na::dim::<V>()),
                    CylinderGeom(ref c) => cylinder_volume(&c.half_height(), &c.radius(), na::dim::<V>()),
                    CapsuleGeom(ref c)  => capsule_volume(&c.half_height(), &c.radius(), na::dim::<V>()),
                }
            }
        }
    }
    */

    #[inline]
    fn mass_properties(&self, density: &N) -> (N, V, II) {
        match *self {
            PlaneGeom(ref p)    => p.mass_properties(density),
            CompoundGeom(ref c) => {
                let mut mtot: N  = na::zero();
                let mut itot: II = na::zero();
                let mut ctot: V  = na::zero();

                for &(ref m, ref s) in c.get().shapes().iter() {
                    let (mpart, cpart, ipart): (N, V, II) = s.mass_properties(density);
                    mtot = mtot + mpart;
                    itot = itot + ipart.to_world_space(m).to_relative_wrt_point(&mpart, &m.translation());
                    ctot = ctot + cpart * mpart;
                }

                ctot = ctot / mtot;

                (mtot, ctot, itot)
            },
            ImplicitGeom(ref i) => {
                match *i {
                    BallGeom(ref b)     => b.mass_properties(density),
                    BoxGeom(ref b)      => b.mass_properties(density),
                    ConeGeom(ref c)     => c.mass_properties(density),
                    CylinderGeom(ref c) => c.mass_properties(density),
                    CapsuleGeom(ref c)  => c.mass_properties(density),
                }
            }
        }
    }
}

#[inline]
fn ball_volume<N: Real + Num + Cast<f32>>(radius: &N, dim: uint) -> N {
    let _pi: N = Real::pi();
    _pi * radius.pow(&Cast::from(dim as f32))
}

impl<N:  Real + Num + Cast<f32> + Clone,
     V:  Zero + Dim,
     II: Zero + Indexable<(uint, uint), N>>
Volumetric<N, V, II> for Ball<N> {
    /*
    #[inline]
    fn volume(&self) -> N {
        ball_volume(&self.radius(), na::dim::<V>())
    }
    */

    fn mass_properties(&self, density: &N) -> (N, V, II) {
        let volume = ball_volume(&self.radius(), na::dim::<V>());
        let mass   = volume * *density;

        let dim = na::dim::<V>();

        if dim == 2 {
            let diag = self.radius() * self.radius() * mass / Cast::from(2.0);

            let mut res: II = na::zero();

            res.set((0, 0), diag);

            (mass, na::zero(), res)
        }
        else if dim == 3 {
            let _0: N  = na::zero();
            let diag: N = mass                  *
                          Cast::from(2.0 / 5.0) *
                          self.radius()         *
                          self.radius();

            let mut res: II = na::zero();

            res.set((0, 0), diag.clone());
            res.set((1, 1), diag.clone());
            res.set((2, 2), diag.clone());

            (mass, na::zero(), res)
        }
        else {
            fail!("Inertia tensor for n-dimensional balls, n > 3, is not implemented.")
        }
    }
}

impl<N:  Zero + One + Cast<f32> + Num + Clone,
     V:  Clone + VecExt<N>,
     II: Zero + Indexable<(uint, uint), N>>
Volumetric<N, V, II> for Box<N, V> {
    /*
    fn volume(&self) -> N {
        box_volume(&self.half_extents())
    }
    */

    fn mass_properties(&self, density: &N) -> (N, V, II) {
        let mass = box_volume(&self.half_extents()) * *density;
        let dim  = na::dim::<V>();

        if dim == 2 {
            let _2: N   = Cast::from(2.0);
            let _i12: N = Cast::from(1.0 / 12.0);
            let w       = _i12 * mass * _2 * _2;
            let ix      = w * self.half_extents().at(0) * self.half_extents().at(0);
            let iy      = w * self.half_extents().at(1) * self.half_extents().at(1);

            let mut res: II = na::zero();

            res.set((0, 0), ix + iy);

            (mass, na::zero(), res)
        }
        else if dim == 3 {
            let _0: N   = na::zero();
            let _2: N   = Cast::from(2.0);
            let _i12: N = Cast::from(1.0 / 12.0);
            let w       = _i12 * mass * _2 * _2;
            let ix      = w * self.half_extents().at(0) * self.half_extents().at(0);
            let iy      = w * self.half_extents().at(1) * self.half_extents().at(1);
            let iz      = w * self.half_extents().at(2) * self.half_extents().at(2);

            let mut res: II = na::zero();

            res.set((0, 0), iy + iz);
            res.set((1, 1), ix + iz);
            res.set((2, 2), ix + iy);

            (mass, na::zero(), res)
        }
        else {
            fail!("Inertia tensor for n-dimensional boxes, n > 3, is not implemented.")
        }
    }
}

#[inline]
fn box_volume<N:  Zero + One + Cast<f32> + Num + Clone,
              V:  Clone + VecExt<N>>(
              half_extents: &V)
              -> N {
    let mut res: N = na::one();

    for half_extent in half_extents.iter() {
        res = res * *half_extent * Cast::from(2.0)
    }

    res
}

impl<N:  Zero + One + Cast<f32> + Num + Real + Clone,
     V:  Zero + Dim,
     II: Zero + Indexable<(uint, uint), N>>
Volumetric<N, V, II> for Cylinder<N> {
    /*
    fn volume(&self) -> N {
        let dim = na::dim::<V>();
        cylinder_volume(&self.half_height(), &self.radius(), dim)
    }
    */

    fn mass_properties(&self, density: &N) -> (N, V, II) {
        let dim  = na::dim::<V>();
        let mass = cylinder_volume(&self.half_height(), &self.radius(), dim) * *density;

        if dim == 2 {
            // same as the box
            let _2:   N = Cast::from(2.0);
            let _i12: N = Cast::from(1.0 / 12.0);
            let w       = _i12 * mass * _2 * _2;
            let ix      = w * self.half_height() * self.half_height();
            let iy      = w * self.radius() * self.radius();

            let mut res: II = na::zero();

            res.set((0, 0), ix + iy);

            (mass, na::zero(), res)
        }
        else if dim == 3 {
            let sq_radius = self.radius() * self.radius();
            let sq_height = self.half_height() * self.half_height() * Cast::from(4.0);
            let off_principal = mass * (sq_radius * Cast::from(3.0) + sq_height) / Cast::from(12.0);

            let mut res: II = na::zero();

            res.set((0, 0), mass * sq_radius / Cast::from(2.0));
            res.set((1, 1), off_principal.clone());
            res.set((2, 2), off_principal);

            (mass, na::zero(), res)
        }
        else {
            fail!("Inertia tensor for n-dimensional cylinder, n > 3, is not implemented.")
        }
    }
}

#[inline]
fn cylinder_volume<N: Zero + One + Cast<f32> + Num + Real + Clone>(
                   half_height: &N,
                   radius:      &N,
                   dim:         uint)
                   -> N {
    if dim == 2 {
        // same as a rectangle
        half_height * *radius * Cast::from(4.0)
    }
    else if dim == 3 {
        half_height * *radius * *radius * Real::pi() * Cast::from(2.0)
    }
    else {
        fail!("Volume for n-dimensional cylinders, n > 3, is not implemented.")
    }
}

impl<N:  Zero + One + Cast<f32> + Num + Real + Clone,
     V:  Zero + Dim,
     II: Zero + Indexable<(uint, uint), N>>
Volumetric<N, V, II> for Capsule<N> {
    /*
    fn volume(&self) -> N {
        let dim = na::dim::<V>();
        capsule_volume(&self.half_height(), &self.radius(), dim)
    }
    */

    fn mass_properties(&self, _: &N) -> (N, V, II) {
        fail!("Not yet implemented.")
    }
}

#[inline]
fn capsule_volume<N: Zero + One + Cast<f32> + Num + Real + Clone>(
                  half_height: &N,
                  radius:      &N,
                  dim:         uint)
                  -> N {
    cylinder_volume(half_height, radius, dim) + ball_volume(radius, dim)
}

impl<N:  Zero + One + Cast<f32> + Num + Real + Clone,
     V:  Zero + Indexable<uint, N> + Dim,
     II: Zero + Indexable<(uint, uint), N>>
Volumetric<N, V, II> for Cone<N> {
    /*
    fn volume(&self) -> N {
        let dim = na::dim::<V>();

        cone_volume(&self.half_height(), &self.radius(), dim)
    }
    */

    fn mass_properties(&self, density: &N) -> (N, V, II) {
        let dim  = na::dim::<V>();
        let mass = cone_volume(&self.half_height(), &self.radius(), dim) * *density;

        if dim == 2 {
            // FIXME: not sure about that…
            let mut res: II = na::zero();

            res.set(
                (0, 0),
                self.radius() * self.half_height() * self.half_height() * self.half_height()
                              / Cast::from(3.0)
            );

            let mut center: V = na::zero();
            center.set(0, -self.half_height() / Cast::from(2.0));

            (mass, center, res)
        }
        else if dim == 3 {
            let m_sq_radius = mass * self.radius() * self.radius();
            let m_sq_height = mass * self.half_height() * self.half_height() *
                                     Cast::from(4.0);
            let off_principal = m_sq_radius * Cast::from(3.0 / 20.0) +
                                m_sq_height * Cast::from(3.0 / 5.0);

            let principal = m_sq_radius * Cast::from(3.0 / 10.0);

            let mut res: II = na::zero();

            res.set((0, 0), principal);
            res.set((1, 1), off_principal.clone());
            res.set((2, 2), off_principal);

            let mut center: V = na::zero();
            center.set(0, -self.half_height() / Cast::from(2.0));

            (mass, center, res)
        }

        else {
            fail!("Inertia tensor for n-dimensional cone, n > 3, is not implemented.")
        }
    }
}

#[inline]
fn cone_volume<N:  Zero + One + Cast<f32> + Num + Real + Clone>(
               half_height: &N,
               radius:      &N,
               dim:         uint)
               -> N {
    if dim == 2 {
        // same as a isosceles triangle
        *radius * *half_height * Cast::from(2.0)
    }
    else if dim == 3 {
        *radius * *radius * Real::pi() * *half_height * Cast::from(2.0 / 3.0)
    }
    else {
        fail!("Volume for n-dimensional cone, n > 3, is not implemented.")
    }
}

impl<N: Zero, V: Zero, II: Zero> Volumetric<N, V, II> for Plane<N, V> {
    /*
    #[inline]
    fn volume(&self) -> N {
        na::zero()
    }
    */

    #[inline]
    fn mass_properties(&self, _: &N) -> (N, V, II) {
        (na::zero(), na::zero(), na::zero())
    }
}
