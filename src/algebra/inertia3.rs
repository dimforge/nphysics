use std::ops::{Add, AddAssign, Mul};

use na::{self, Isometry3, Matrix3, Matrix6, RealField, U3};
use crate::algebra::{Force3, Velocity3};

/// The inertia of a rigid body grouping both its mass and its angular inertia.
#[derive(Clone, Copy, Debug)]
pub struct Inertia3<N: RealField> {
    /// The linear part (mass) of the inertia.
    pub linear: N,
    /// The angular inertia.
    pub angular: Matrix3<N>,
}

impl<N: RealField> Inertia3<N> {
    /// Creates an inertia from its linear and angular components.
    pub fn new(linear: N, angular: Matrix3<N>) -> Self {
        Inertia3 { linear, angular }
    }

    /// Creates an inertia from its linear and angular components.
    pub fn new_with_angular_matrix(linear: N, angular: Matrix3<N>) -> Self {
        Self::new(linear, angular)
    }

    /// Get the mass.
    pub fn mass(&self) -> N {
        self.linear
    }

    /// Create a zero inertia.
    pub fn zero() -> Self {
        Inertia3::new(na::zero(), na::zero())
    }

    /// Get the angular inertia tensor.
    #[inline]
    pub fn angular_matrix(&self) -> &Matrix3<N> {
        &self.angular
    }

    /// Convert the inertia into a matrix where the mass is represented as a 3x3
    /// diagonal matrix on the upper-left corner, and the angular part as a 3x3
    /// matrix on the lower-rigth corner.
    pub fn to_matrix(&self) -> Matrix6<N> {
        let mut res = Matrix6::zeros();
        res.fixed_slice_mut::<U3, U3>(3, 3).copy_from(&self.angular);

        res.m11 = self.linear;
        res.m22 = self.linear;
        res.m33 = self.linear;

        res
    }

    /// Compute the inertia on the given coordinate frame.
    pub fn transformed(&self, i: &Isometry3<N>) -> Self {
        let rot = i.rotation.to_rotation_matrix();
        Inertia3::new(self.linear, rot * self.angular * rot.inverse())
    }

    /// Inverts this inetia matrix.
    ///
    /// Sets the angular part to zero if it is not invertible.
    pub fn inverse(&self) -> Self {
        let inv_mass = if self.linear.is_zero() { N::zero() } else { N::one() / self.linear };
        let inv_angular = self.angular.try_inverse().unwrap_or_else(na::zero);
        Inertia3::new(inv_mass, inv_angular)
    }
}

impl<N: RealField> Add<Inertia3<N>> for Inertia3<N> {
    type Output = Inertia3<N>;

    #[inline]
    fn add(self, rhs: Inertia3<N>) -> Inertia3<N> {
        Inertia3::new(self.linear + rhs.linear, self.angular + rhs.angular)
    }
}

impl<N: RealField> AddAssign<Inertia3<N>> for Inertia3<N> {
    #[inline]
    fn add_assign(&mut self, rhs: Inertia3<N>) {
        self.linear += rhs.linear;
        self.angular += rhs.angular;
    }
}

impl<N: RealField> Mul<Velocity3<N>> for Inertia3<N> {
    type Output = Force3<N>;

    #[inline]
    fn mul(self, rhs: Velocity3<N>) -> Force3<N> {
        Force3::new(rhs.linear * self.linear, self.angular * rhs.angular)
    }
}

// NOTE: This is meaningful when `self` is the inverse inertia.
impl<N: RealField> Mul<Force3<N>> for Inertia3<N> {
    type Output = Velocity3<N>;

    #[inline]
    fn mul(self, rhs: Force3<N>) -> Velocity3<N> {
        Velocity3::new(rhs.linear * self.linear, self.angular * rhs.angular)
    }
}
