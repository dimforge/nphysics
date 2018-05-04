use std::mem;
use std::ops::{Add, AddAssign, Mul};

use na::{self, Isometry2, Matrix1, Matrix3, Real, Vector3};
use algebra::{Force2, Velocity2};

/// The inertia of a rigid body grouping both its mass and its angular inertia.
#[derive(Clone, Copy, Debug)]
pub struct Inertia2<N: Real> {
    /// The linear part (mass) of the inertia.
    pub linear: N,
    /// The angular inertia.
    pub angular: N,
}

impl<N: Real> Inertia2<N> {
    /// Creates an inertia from its linear and angular components.
    pub fn new(linear: N, angular: N) -> Self {
        Inertia2 { linear, angular }
    }

    /// Creates an inertia from its linear and angular components.
    pub fn new_with_angular_matrix(linear: N, angular: Matrix1<N>) -> Self {
        Self::new(linear, angular.x)
    }

    /// Get the mass.
    pub fn mass(&self) -> N {
        self.linear
    }

    /// Create a zero inertia.
    pub fn zero() -> Self {
        Inertia2::new(na::zero(), na::zero())
    }

    /// Get the angular inertia tensor.
    #[inline]
    pub fn angular_matrix(&self) -> &Matrix1<N> {
        unsafe { mem::transmute(&self.angular) }
    }

    /// Convert the inertia into a matrix where the mass is represented as a 2x2
    /// diagonal matrix on the upper-left corner, and the angular part as a 1x1
    /// matrix on the lower-rigth corner.
    pub fn to_matrix(&self) -> Matrix3<N> {
        let diag = Vector3::new(self.linear, self.linear, self.angular);
        Matrix3::from_diagonal(&diag)
    }

    /// Compute the inertia on the given coordinate frame.
    pub fn transformed(&self, _: &Isometry2<N>) -> Self {
        *self
    }

    /// Inverts this inetia matrix.
    ///
    /// Sets the angular part to zero if it is not invertible.
    pub fn inverse(&self) -> Self {
        let inv_mass = N::one() / self.linear;
        let inv_angular = if self.angular.is_zero() {
            N::zero()
        } else {
            N::one() / self.angular
        };
        Inertia2::new(inv_mass, inv_angular)
    }
}

impl<N: Real> Add<Inertia2<N>> for Inertia2<N> {
    type Output = Inertia2<N>;

    #[inline]
    fn add(self, rhs: Inertia2<N>) -> Inertia2<N> {
        Inertia2::new(self.linear + rhs.linear, self.angular + rhs.angular)
    }
}

impl<N: Real> AddAssign<Inertia2<N>> for Inertia2<N> {
    #[inline]
    fn add_assign(&mut self, rhs: Inertia2<N>) {
        self.linear += rhs.linear;
        self.angular += rhs.angular;
    }
}

impl<N: Real> Mul<Velocity2<N>> for Inertia2<N> {
    type Output = Force2<N>;

    #[inline]
    fn mul(self, rhs: Velocity2<N>) -> Force2<N> {
        Force2::new(rhs.linear * self.linear, self.angular * rhs.angular)
    }
}

// NOTE: This is meaningful when `self` is the inverse inertia.
impl<N: Real> Mul<Force2<N>> for Inertia2<N> {
    type Output = Velocity2<N>;

    #[inline]
    fn mul(self, rhs: Force2<N>) -> Velocity2<N> {
        Velocity2::new(rhs.linear * self.linear, self.angular * rhs.angular)
    }
}
