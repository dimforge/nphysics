use std::mem;
use std::ops::{Add, AddAssign, Mul, Sub, SubAssign};
use na::{self, Isometry2, Real, Rotation2, U3, Vector, Vector1, Vector2, Vector3};
use na::storage::Storage;

#[derive(Copy, Clone, Debug)]
pub struct Velocity2<N: Real> {
    pub linear: Vector2<N>,
    pub angular: N,
}

impl<N: Real> Velocity2<N> {
    #[inline]
    pub fn new(linear: Vector2<N>, angular: N) -> Self {
        Velocity2 { linear, angular }
    }

    #[inline]
    pub fn new_with_vectors(linear: Vector2<N>, angular: Vector1<N>) -> Self {
        Self::new(linear, angular.x)
    }

    #[inline]
    pub fn angular(w: N) -> Self {
        Velocity2::new(na::zero(), w)
    }

    #[inline]
    pub fn linear(vx: N, vy: N) -> Self {
        Velocity2::new(Vector2::new(vx, vy), N::zero())
    }

    #[inline]
    pub fn zero() -> Self {
        Self::new(na::zero(), N::zero())
    }

    #[inline]
    pub fn angular_vector(&self) -> Vector1<N> {
        Vector1::new(self.angular)
    }

    /// This twist seen as a slice.
    #[inline]
    pub fn as_slice(&self) -> &[N] {
        self.as_vector().as_slice()
    }

    /// This twist seen as a slice.
    #[inline]
    pub fn as_mut_slice(&mut self) -> &mut [N] {
        self.as_vector_mut().as_mut_slice()
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

    #[inline]
    pub fn from_vector<S: Storage<N, U3>>(data: &Vector<N, U3, S>) -> Self {
        Self::new(Vector2::new(data[0], data[1]), data[2])
    }

    #[inline]
    pub fn from_slice(data: &[N]) -> Self {
        Self::new(Vector2::new(data[0], data[1]), data[2])
    }

    #[inline]
    pub fn to_transform(&self) -> Isometry2<N> {
        Isometry2::new(self.linear, self.angular)
    }

    #[inline]
    pub fn frame_velocity(&self, frame: &Isometry2<N>) -> Self {
        Self::new(
            self.linear + frame.translation.vector * self.angular,
            self.angular,
        )
    }

    // XXX: not a good name
    #[inline]
    pub fn shift(&self, shift: &Vector2<N>) -> Self {
        Self::new(self.linear + shift * self.angular, self.angular)
    }

    #[inline]
    pub fn rotated(&self, rot: &Rotation2<N>) -> Self {
        Self::new(rot * self.linear, self.angular)
    }

    #[inline]
    pub fn transformed(&self, iso: &Isometry2<N>) -> Self {
        Self::new(iso * self.linear, self.angular)
    }
}

impl<N: Real> Add<Velocity2<N>> for Velocity2<N> {
    type Output = Self;

    #[inline]
    fn add(self, rhs: Self) -> Self {
        Velocity2::new(self.linear + rhs.linear, self.angular + rhs.angular)
    }
}

impl<N: Real> AddAssign<Velocity2<N>> for Velocity2<N> {
    #[inline]
    fn add_assign(&mut self, rhs: Self) {
        self.linear += rhs.linear;
        self.angular += rhs.angular;
    }
}

impl<N: Real> Sub<Velocity2<N>> for Velocity2<N> {
    type Output = Self;

    #[inline]
    fn sub(self, rhs: Self) -> Self {
        Velocity2::new(self.linear - rhs.linear, self.angular - rhs.angular)
    }
}

impl<N: Real> SubAssign<Velocity2<N>> for Velocity2<N> {
    #[inline]
    fn sub_assign(&mut self, rhs: Self) {
        self.linear -= rhs.linear;
        self.angular -= rhs.angular;
    }
}

impl<N: Real> Mul<N> for Velocity2<N> {
    type Output = Self;

    #[inline]
    fn mul(self, rhs: N) -> Self {
        Velocity2::new(self.linear * rhs, self.angular * rhs)
    }
}
