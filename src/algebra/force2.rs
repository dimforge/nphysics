use std::mem;
use std::ops::{Add, AddAssign, Mul, Sub, SubAssign};
use na::{self, Point2, Real, U3, Vector, Vector1, Vector2, Vector3};
use na::storage::Storage;

#[derive(Copy, Clone, Debug)]
pub struct Force2<N: Real> {
    pub linear: Vector2<N>,
    pub angular: N,
}

impl<N: Real> Force2<N> {
    #[inline]
    pub fn new(linear: Vector2<N>, angular: N) -> Self {
        Force2 { linear, angular }
    }

    #[inline]
    pub fn zero() -> Self {
        Self::new(na::zero(), N::zero())
    }

    #[inline]
    pub fn from_slice(data: &[N]) -> Self {
        Self::new(Vector2::new(data[0], data[1]), data[2])
    }

    #[inline]
    pub fn from_vector<S: Storage<N, U3>>(data: &Vector<N, U3, S>) -> Self {
        Self::new(Vector2::new(data[0], data[1]), data[2])
    }

    #[inline]
    pub fn torque(torque: N) -> Self {
        Self::new(na::zero(), torque)
    }

    #[inline]
    pub fn torque_from_vector(torque: Vector1<N>) -> Self {
        Self::new(na::zero(), torque.x)
    }

    #[inline]
    pub fn linear_force_at_point(linear: Vector2<N>, point: &Point2<N>) -> Self {
        Self::new(linear, point.coords.perp(&linear))
    }

    #[inline]
    pub fn torque_at_point(torque: N, point: &Point2<N>) -> Self {
        Self::new(point.coords * -torque, torque)
    }

    #[inline]
    pub fn torque_from_vector_at_point(torque: Vector1<N>, point: &Point2<N>) -> Self {
        Self::torque_at_point(torque.x, point)
    }

    /// This twist seen as a slice.
    #[inline]
    pub fn as_slice(&self) -> &[N] {
        self.as_vector().as_slice()
    }

    /// This twist seen as a vector. The linear components are stored first.
    #[inline]
    pub fn as_vector(&self) -> &Vector3<N> {
        unsafe { mem::transmute(self) }
    }

    /// This twist seen as a mutable vector. The linear components are stored first.
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
