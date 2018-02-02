use std::mem;
use std::ops::{Add, AddAssign, Mul, Sub, SubAssign};
use na::{self, Isometry3, Real, U6, UnitQuaternion, Vector, Vector3, Vector6};
use na::storage::Storage;

#[derive(Copy, Clone, Debug)]
pub struct Velocity3<N: Real> {
    pub linear: Vector3<N>,
    pub angular: Vector3<N>,
}

impl<N: Real> Velocity3<N> {
    #[inline]
    pub fn new(linear: Vector3<N>, angular: Vector3<N>) -> Self {
        Velocity3 { linear, angular }
    }

    #[inline]
    pub fn new_with_vectors(linear: Vector3<N>, angular: Vector3<N>) -> Self {
        Self::new(linear, angular)
    }

    #[inline]
    pub fn angular(wx: N, wy: N, wz: N) -> Self {
        Velocity3::new(na::zero(), Vector3::new(wx, wy, wz))
    }

    #[inline]
    pub fn linear(vx: N, vy: N, vz: N) -> Self {
        Velocity3::new(Vector3::new(vx, vy, vz), na::zero())
    }

    #[inline]
    pub fn zero() -> Self {
        Self::new(na::zero(), na::zero())
    }

    #[inline]
    pub fn angular_vector(&self) -> Vector3<N> {
        self.angular
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
    pub fn as_vector(&self) -> &Vector6<N> {
        unsafe { mem::transmute(self) }
    }

    /// This twist seen as a mutable vector. The linear components are stored first.
    #[inline]
    pub fn as_vector_mut(&mut self) -> &mut Vector6<N> {
        unsafe { mem::transmute(self) }
    }

    #[inline]
    pub fn from_vector<S: Storage<N, U6>>(data: &Vector<N, U6, S>) -> Self {
        Self::new(
            Vector3::new(data[0], data[1], data[2]),
            Vector3::new(data[3], data[4], data[5]),
        )
    }

    #[inline]
    pub fn from_slice(data: &[N]) -> Self {
        Self::new(
            Vector3::new(data[0], data[1], data[2]),
            Vector3::new(data[3], data[4], data[5]),
        )
    }

    #[inline]
    pub fn to_transform(&self) -> Isometry3<N> {
        Isometry3::new(self.linear, self.angular)
    }

    #[inline]
    pub fn frame_velocity(&self, frame: &Isometry3<N>) -> Self {
        Self::new(
            self.linear + self.angular.cross(&frame.translation.vector),
            self.angular,
        )
    }

    // XXX: not a good name
    #[inline]
    pub fn shift(&self, shift: &Vector3<N>) -> Self {
        Self::new(self.linear + self.angular.cross(&shift), self.angular)
    }

    #[inline]
    pub fn transformed(&self, iso: &Isometry3<N>) -> Self {
        Self::new(iso * self.linear, iso * self.angular)
    }

    #[inline]
    pub fn rotated(&self, rot: &UnitQuaternion<N>) -> Self {
        Self::new(rot * self.linear, rot * self.angular)
    }
}

impl<N: Real> Add<Velocity3<N>> for Velocity3<N> {
    type Output = Self;

    #[inline]
    fn add(self, rhs: Self) -> Self {
        Velocity3::new(self.linear + rhs.linear, self.angular + rhs.angular)
    }
}

impl<N: Real> AddAssign<Velocity3<N>> for Velocity3<N> {
    #[inline]
    fn add_assign(&mut self, rhs: Self) {
        self.linear += rhs.linear;
        self.angular += rhs.angular;
    }
}

impl<N: Real> Sub<Velocity3<N>> for Velocity3<N> {
    type Output = Self;

    #[inline]
    fn sub(self, rhs: Self) -> Self {
        Velocity3::new(self.linear - rhs.linear, self.angular - rhs.angular)
    }
}

impl<N: Real> SubAssign<Velocity3<N>> for Velocity3<N> {
    #[inline]
    fn sub_assign(&mut self, rhs: Self) {
        self.linear -= rhs.linear;
        self.angular -= rhs.angular;
    }
}

impl<N: Real> Mul<N> for Velocity3<N> {
    type Output = Self;

    #[inline]
    fn mul(self, rhs: N) -> Self {
        Velocity3::new(self.linear * rhs, self.angular * rhs)
    }
}
