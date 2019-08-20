use na::{RealField, Unit};
use crate::math::{Vector, Point};


/// Kernel functions for performing approximations within the PBF method.
pub trait SPHKernel {
    /// Evaluates the kernel for the given scalar `r` and the reference support length `h`.
    fn scalar_apply<N: RealField>(r: N, h: N) -> N;
    /// Evaluates the kernel derivative for the given scalar `r` and the reference support length `h`.
    fn scalar_apply_diff<N: RealField>(r: N, h: N) -> N;

    /// Evaluate the kernel for the given vector.
    fn apply<N: RealField>(v: Vector<N>, h: N) -> N {
        Self::scalar_apply(v.norm(), h)
    }

    /// Differential wrt. the coordinates of `v`.
    fn apply_diff<N: RealField>(v: Vector<N>, h: N) -> Vector<N> {
        if let Some((dir, norm)) = Unit::try_new_and_get(v, N::default_epsilon()) {
            *dir * Self::scalar_apply_diff(norm, h)
        } else {
            Vector::zeros()
        }
    }

    /// Evaluate the kernel for the vector equal to `p1 - p2`.
    fn points_apply<N: RealField>(p1: &Point<N>, p2: &Point<N>, h: N) -> N {
        Self::apply(p1 - p2, h)
    }

    /// Differential wrt. the coordinates of `p1`.
    fn points_apply_diff1<N: RealField>(p1: &Point<N>, p2: &Point<N>, h: N) -> Vector<N> {
        Self::apply_diff(p1 - p2, h)
    }

    /// Differential wrt. the coordinates of `p2`.
    fn points_apply_diff2<N: RealField>(p1: &Point<N>, p2: &Point<N>, h: N) -> Vector<N> {
        -Self::apply_diff(p1 - p2, h)
    }
}

/// https://pysph.readthedocs.io/en/latest/reference/kernels.html
pub struct CubicSplineKernel;
/// Particle-Based Fluid Simulation for Interactive Applications, Müller et al.
pub struct Poly6Kernel;
/// Particle-Based Fluid Simulation for Interactive Applications, Müller et al.
pub struct SpikyKernel;
/// Particle-Based Fluid Simulation for Interactive Applications, Müller et al.
pub struct ViscosityKernel;

impl SPHKernel for CubicSplineKernel {
    fn scalar_apply<N: RealField>(r: N, h: N) -> N {
        assert!(r >= N::zero());

        let q = r / h;
        #[cfg(feature = "dim2")]
            let normalizer = na::convert::<_, N>(10.0 / 7.0) / (N::pi() * h * h);
        #[cfg(feature = "dim3")]
            let normalizer = N::one() / (N::pi() * h * h * h);

        let _2: N = na::convert(2.0);
        let _3: N = na::convert(3.0);
        let rhs = if q <= N::one() {
            N::one() - _3 / _2 * q * q * (N::one() - q / _2)
        } else if q <= _2 {
            (_2 - q).powi(3) / na::convert(4.0)
        } else {
            N::zero()
        };

        normalizer * rhs
    }

    fn scalar_apply_diff<N: RealField>(r: N, h: N) -> N {
        assert!(r >= N::zero());

        let q = r / h;
        #[cfg(feature = "dim2")]
            let normalizer = na::convert::<_, N>(10.0 / 7.0) / (N::pi() * h * h);
        #[cfg(feature = "dim3")]
            let normalizer = N::one() / (N::pi() * h * h * h);

        let _2: N = na::convert(2.0);
        let _3: N = na::convert(3.0);
        let rhs = if q <= N::one() {
            -_3 * q * (N::one() - q * na::convert(3.0 / 4.0))
        } else if q <= _2 {
            -(_2 - q).powi(2) * na::convert(3.0 / 4.0)
        } else {
            N::zero()
        };

        normalizer * rhs
    }
}


impl SPHKernel for Poly6Kernel {
    fn scalar_apply<N: RealField>(r: N, h: N) -> N {
        assert!(r >= N::zero());

        #[cfg(feature = "dim2")]
            let normalizer = na::convert::<_, N>(4.0) / (N::pi() * h.powi(8));
        #[cfg(feature = "dim3")]
            let normalizer = na::convert::<_, N>(315.0 / 64.0) / (N::pi() * h.powi(9));

        if r <= h {
            normalizer * (h * h - r * r).powi(3)
        } else {
            N::zero()
        }
    }

    fn scalar_apply_diff<N: RealField>(r: N, h: N) -> N {
        assert!(r >= N::zero());

        #[cfg(feature = "dim2")]
            let normalizer = na::convert::<_, N>(4.0) / (N::pi() * h.powi(8));
        #[cfg(feature = "dim3")]
            let normalizer = na::convert::<_, N>(315.0 / 64.0) / (N::pi() * h.powi(9));

        if r <= h {
            normalizer * (h * h - r * r).powi(2) * r * na::convert(-6.0)
        } else {
            N::zero()
        }
    }
}


impl SPHKernel for SpikyKernel {
    fn scalar_apply<N: RealField>(r: N, h: N) -> N {
        assert!(r >= N::zero());

        #[cfg(feature = "dim2")]
            let normalizer = na::convert::<_, N>(10.0) / (N::pi() * h.powi(5));
        #[cfg(feature = "dim3")]
            let normalizer = na::convert::<_, N>(15.0) / (N::pi() * h.powi(6));

        if r <= h {
            normalizer * (h - r).powi(3)
        } else {
            N::zero()
        }
    }

    fn scalar_apply_diff<N: RealField>(r: N, h: N) -> N {
        assert!(r >= N::zero());

        #[cfg(feature = "dim2")]
            let normalizer = na::convert::<_, N>(10.0) / (N::pi() * h.powi(5));
        #[cfg(feature = "dim3")]
            let normalizer = na::convert::<_, N>(15.0) / (N::pi() * h.powi(6));

        if r <= h {
            -normalizer * (h - r).powi(2) * na::convert(3.0)
        } else {
            N::zero()
        }
    }
}


impl SPHKernel for ViscosityKernel {
    fn scalar_apply<N: RealField>(r: N, h: N) -> N {
        assert!(r >= N::zero());

        let _2: N = na::convert(2.0);
        let _3: N = na::convert(3.0);

        #[cfg(feature = "dim2")]
            let normalizer = na::convert::<_, N>(10.0) / (_3 * N::pi() * h.powi(2));
        #[cfg(feature = "dim3")]
            let normalizer = na::convert::<_, N>(15.0) / (_2 * N::pi() * h.powi(3));


        if r > N::zero() && r <= h {
            let rr_hh = r * r / (h * h);
            normalizer * (rr_hh * (N::one() - r / (_2 * h)) + h / (_2 * r) - N::one())
        } else {
            N::zero()
        }
    }

    fn scalar_apply_diff<N: RealField>(r: N, h: N) -> N {
        assert!(r >= N::zero());

        let _2: N = na::convert(2.0);
        let _3: N = na::convert(3.0);

        #[cfg(feature = "dim2")]
            let normalizer = na::convert::<_, N>(10.0) / (_3 * N::pi() * h.powi(2));
        #[cfg(feature = "dim3")]
            let normalizer = na::convert::<_, N>(15.0) / (_2 * N::pi() * h.powi(3));

        if r > N::zero() && r <= h {
            let rr = r * r;
            let hh = h * h;
            let hhh = hh * h;
            normalizer * (-_3 * rr / (_2 * hhh) + _2 * r / hh - h / (_2 * rr))
        } else {
            N::zero()
        }
    }
}