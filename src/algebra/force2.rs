use na::storage::Storage;
use na::{self, Point2, Real, U3, Vector, Vector1, Vector2, Vector3, Isometry2};
use std::mem;
use std::ops::{Add, AddAssign, Mul, Neg, Sub, SubAssign};

/// A force with a linear and agular (torque) component.
#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct Force2<N: Real> {
    /// The linear force.
    pub linear: Vector2<N>,
    /// The torque.
    pub angular: N,
}

impl<N: Real> Force2<N> {
    /// Creates a force from its linear and angular components.
    #[inline]
    pub fn new(linear: Vector2<N>, angular: N) -> Self {
        Force2 { linear, angular }
    }

    /// A zero force.
    #[inline]
    pub fn zero() -> Self {
        Self::new(na::zero(), N::zero())
    }

    /// Create a force from a slice where the entries 0 and 1 are for the linear part and 2 for the angular part.
    #[inline]
    pub fn from_slice(data: &[N]) -> Self {
        Self::new(Vector2::new(data[0], data[1]), data[2])
    }

    /// Create a force from a vector where the entries 0 and 1 are for the linear part and 2 for the angular part.
    #[inline]
    pub fn from_vector<S: Storage<N, U3>>(data: &Vector<N, U3, S>) -> Self {
        Self::new(Vector2::new(data[0], data[1]), data[2])
    }

    /// Create a pure torque.
    #[inline]
    pub fn torque(torque: N) -> Self {
        Self::new(na::zero(), torque)
    }

    /// Create a pure torque.
    #[inline]
    pub fn torque_from_vector(torque: Vector1<N>) -> Self {
        Self::new(na::zero(), torque.x)
    }

    /// Create a pure linear force.
    #[inline]
    pub fn linear(linear: Vector2<N>) -> Self {
        Self::new(linear, na::zero())
    }

    /// Creates the resultant of a linear force applied at the given point (relative to the center of mass).
    #[inline]
    pub fn linear_at_point(linear: Vector2<N>, point: &Point2<N>) -> Self {
        Self::new(linear, point.coords.perp(&linear))
    }

    /// Creates the resultant of a torque applied at the given point (relative to the center of mass).
    #[inline]
    pub fn torque_at_point(torque: N, point: &Point2<N>) -> Self {
        Self::new(point.coords * -torque, torque)
    }

    /// Creates the resultant of a torque applied at the given point (relative to the center of mass).
    #[inline]
    pub fn torque_from_vector_at_point(torque: Vector1<N>, point: &Point2<N>) -> Self {
        Self::torque_at_point(torque.x, point)
    }

    /// The angular part of the force.
    #[inline]
    pub fn angular_vector(&self) -> Vector1<N> {
        Vector1::new(self.angular)
    }

    #[inline]
    pub fn transform_by(&self, m: &Isometry2<N>) -> Self {
        Self::new(m * self.linear, self.angular)
    }

    /// This force seen as a slice.
    ///
    /// The two first entries contain the linear part and the third entry contais the angular part.
    #[inline]
    pub fn as_slice(&self) -> &[N] {
        self.as_vector().as_slice()
    }

    /// This force seen as a vector.
    ///
    /// The two first entries contain the linear part and the third entry contais the angular part.
    #[inline]
    pub fn as_vector(&self) -> &Vector3<N> {
        unsafe { mem::transmute(self) }
    }

    /// This force seen as a mutable vector.
    ///
    /// The two first entries contain the linear part and the third entry contais the angular part.
    #[inline]
    pub fn as_vector_mut(&mut self) -> &mut Vector3<N> {
        unsafe { mem::transmute(self) }
    }
}

impl<N: Real> Add<Force2<N>> for Force2<N> {
    type Output = Self;

    #[inline]
    fn add(self, rhs: Self) -> Self {
        Force2::new(self.linear + rhs.linear, self.angular + rhs.angular)
    }
}

impl<N: Real> AddAssign<Force2<N>> for Force2<N> {
    #[inline]
    fn add_assign(&mut self, rhs: Self) {
        self.linear += rhs.linear;
        self.angular += rhs.angular;
    }
}

impl<N: Real> Sub<Force2<N>> for Force2<N> {
    type Output = Self;

    #[inline]
    fn sub(self, rhs: Self) -> Self {
        Force2::new(self.linear - rhs.linear, self.angular - rhs.angular)
    }
}

impl<N: Real> SubAssign<Force2<N>> for Force2<N> {
    #[inline]
    fn sub_assign(&mut self, rhs: Self) {
        self.linear -= rhs.linear;
        self.angular -= rhs.angular;
    }
}

impl<N: Real> Mul<N> for Force2<N> {
    type Output = Self;

    #[inline]
    fn mul(self, rhs: N) -> Self {
        Force2::new(self.linear * rhs, self.angular * rhs)
    }
}

impl<N: Real> Neg for Force2<N> {
    type Output = Self;

    #[inline]
    fn neg(self) -> Self {
        Force2::new(-self.linear, -self.angular)
    }
}
