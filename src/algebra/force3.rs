use na::storage::Storage;
use na::{self, Isometry3, Point3, RealField, Vector, Vector3, Vector6, U6};
use std::mem;
use std::ops::{Add, AddAssign, Mul, Neg, Sub, SubAssign};

/// A force with a linear and angular (torque) component.
#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct Force3<N: RealField> {
    /// The linear force.
    pub linear: Vector3<N>,
    /// The linear force.
    pub angular: Vector3<N>,
}

impl<N: RealField> Force3<N> {
    /// Creates a force from its linear and angular components.
    #[inline]
    pub fn new(linear: Vector3<N>, angular: Vector3<N>) -> Self {
        Force3 { linear, angular }
    }

    /// A zero force.
    #[inline]
    pub fn zero() -> Self {
        Self::new(na::zero(), na::zero())
    }

    /// Create a force from a slice where the linear part are stored first.
    #[inline]
    pub fn from_slice(data: &[N]) -> Self {
        Self::new(
            Vector3::new(data[0], data[1], data[2]),
            Vector3::new(data[3], data[4], data[5]),
        )
    }

    /// Create a force from a vector where the linear part are stored first.
    #[inline]
    pub fn from_vector<S: Storage<N, U6>>(data: &Vector<N, U6, S>) -> Self {
        Self::new(
            Vector3::new(data[0], data[1], data[2]),
            Vector3::new(data[3], data[4], data[5]),
        )
    }

    /// Create a pure torque.
    #[inline]
    pub fn torque(torque: Vector3<N>) -> Self {
        Self::new(na::zero(), torque)
    }

    /// Create a pure torque.
    #[inline]
    pub fn torque_from_vector(torque: Vector3<N>) -> Self {
        Self::new(na::zero(), torque)
    }

    /// Creates the resultant of a torque applied at the given point (relative to the center of mass).
    #[inline]
    pub fn torque_at_point(torque: Vector3<N>, point: &Point3<N>) -> Self {
        Self::new(-torque.cross(&point.coords), torque)
    }

    /// Creates the resultant of a torque applied at the given point (relative to the center of mass).
    #[inline]
    pub fn torque_from_vector_at_point(torque: Vector3<N>, point: &Point3<N>) -> Self {
        Self::torque_at_point(torque, point)
    }

    /// Create a pure linear force.
    #[inline]
    pub fn linear(linear: Vector3<N>) -> Self {
        Self::new(linear, na::zero())
    }

    /// Creates the resultant of a linear force applied at the given point (relative to the center of mass).
    #[inline]
    pub fn linear_at_point(linear: Vector3<N>, point: &Point3<N>) -> Self {
        Self::new(linear, point.coords.cross(&linear))
    }

    /// The angular part of the force.
    #[inline]
    pub fn angular_vector(&self) -> Vector3<N> {
        self.angular
    }

    /// This force seen as a slice.
    ///
    /// The two first entries contain the linear part and the third entry contais the angular part.
    #[inline]
    pub fn as_slice(&self) -> &[N] {
        self.as_vector().as_slice()
    }

    /// Apply the given transformation to this force.
    #[inline]
    pub fn transform_by(&self, m: &Isometry3<N>) -> Self {
        Self::new(m * self.linear, m * self.angular)
    }

    /// This force seen as a vector.
    ///
    /// The linear part of the force are stored first.
    #[inline]
    pub fn as_vector(&self) -> &Vector6<N> {
        unsafe { mem::transmute(self) }
    }

    /// This force seen as a mutable vector.
    ///
    /// The linear part of the force are stored first.
    #[inline]
    pub fn as_vector_mut(&mut self) -> &mut Vector6<N> {
        unsafe { mem::transmute(self) }
    }
}

impl<N: RealField> Add<Force3<N>> for Force3<N> {
    type Output = Self;

    #[inline]
    fn add(self, rhs: Self) -> Self {
        Force3::new(self.linear + rhs.linear, self.angular + rhs.angular)
    }
}

impl<N: RealField> AddAssign<Force3<N>> for Force3<N> {
    #[inline]
    fn add_assign(&mut self, rhs: Self) {
        self.linear += rhs.linear;
        self.angular += rhs.angular;
    }
}

impl<N: RealField> Sub<Force3<N>> for Force3<N> {
    type Output = Self;

    #[inline]
    fn sub(self, rhs: Self) -> Self {
        Force3::new(self.linear - rhs.linear, self.angular - rhs.angular)
    }
}

impl<N: RealField> SubAssign<Force3<N>> for Force3<N> {
    #[inline]
    fn sub_assign(&mut self, rhs: Self) {
        self.linear -= rhs.linear;
        self.angular -= rhs.angular;
    }
}

impl<N: RealField> Mul<N> for Force3<N> {
    type Output = Self;

    #[inline]
    fn mul(self, rhs: N) -> Self {
        Force3::new(self.linear * rhs, self.angular * rhs)
    }
}

impl<N: RealField> Neg for Force3<N> {
    type Output = Self;

    #[inline]
    fn neg(self) -> Self {
        Force3::new(-self.linear, -self.angular)
    }
}
