use na::storage::Storage;
use na::{self, Isometry3, RealField, UnitQuaternion, Vector, Vector3, Vector6, U6};
use std::mem;
use std::ops::{Add, AddAssign, Mul, Sub, SubAssign};

/// A velocity structure combining both the linear angular velocities of a point.
#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct Velocity3<N: RealField> {
    /// The linear velocity.
    pub linear: Vector3<N>,
    /// The angular velocity.
    pub angular: Vector3<N>,
}

impl<N: RealField> Velocity3<N> {
    /// Create velocity from its linear and angular parts.
    #[inline]
    pub fn new(linear: Vector3<N>, angular: Vector3<N>) -> Self {
        Velocity3 { linear, angular }
    }

    /// Create velocity from its linear and angular parts.
    #[inline]
    pub fn from_vectors(linear: Vector3<N>, angular: Vector3<N>) -> Self {
        Self::new(linear, angular)
    }

    /// Create a purely angular velocity.
    #[inline]
    pub fn angular(wx: N, wy: N, wz: N) -> Self {
        Velocity3::new(na::zero(), Vector3::new(wx, wy, wz))
    }

    /// Create a purely linear velocity.
    #[inline]
    pub fn linear(vx: N, vy: N, vz: N) -> Self {
        Velocity3::new(Vector3::new(vx, vy, vz), na::zero())
    }

    /// Computes the velocity required to move from `start` to `end` in the given `time`.
    pub fn between_positions(start: &Isometry3<N>, end: &Isometry3<N>, time: N) -> Self {
        let delta = end / start;
        let linear = delta.translation.vector / time;
        let angular = delta.rotation.scaled_axis() / time;
        Self::new(linear, angular)
    }

    /// Compute the displacement due to this velocity integrated during the time `dt`.
    pub fn integrate(&self, dt: N) -> Isometry3<N> {
        (*self * dt).to_transform()
    }

    /// Compute the displacement due to this velocity integrated during a time equal to `1.0`.
    ///
    /// This is equivalent to `self.integrate(1.0)`.
    pub fn to_transform(&self) -> Isometry3<N> {
        Isometry3::new(self.linear, self.angular)
    }

    /// Create a zero velocity.
    #[inline]
    pub fn zero() -> Self {
        Self::new(na::zero(), na::zero())
    }

    /// The angular part of the velocity.
    #[inline]
    pub fn angular_vector(&self) -> Vector3<N> {
        self.angular
    }

    /// This velocity seen as a slice.
    ///
    /// The linear part is stored first.
    #[inline]
    pub fn as_slice(&self) -> &[N] {
        self.as_vector().as_slice()
    }

    /// This velocity seen as a mutable slice.
    ///
    /// The linear part is stored first.
    #[inline]
    pub fn as_mut_slice(&mut self) -> &mut [N] {
        self.as_vector_mut().as_mut_slice()
    }

    /// This velocity seen as a vector.
    ///
    /// The linear part is stored first.    #[inline]
    pub fn as_vector(&self) -> &Vector6<N> {
        unsafe { mem::transmute(self) }
    }

    /// This velocity seen as a mutable vector.
    ///
    /// The linear part is stored first.    #[inline]
    pub fn as_vector_mut(&mut self) -> &mut Vector6<N> {
        unsafe { mem::transmute(self) }
    }

    /// Create a velocity from a vector.
    ///
    /// The linear part of the velocity is expected to be first inside of the input vector.
    #[inline]
    pub fn from_vector<S: Storage<N, U6>>(data: &Vector<N, U6, S>) -> Self {
        Self::new(
            Vector3::new(data[0], data[1], data[2]),
            Vector3::new(data[3], data[4], data[5]),
        )
    }

    /// Create a velocity from a slice.
    ///
    /// The linear part of the velocity is expected to be first inside of the input slice.
    #[inline]
    pub fn from_slice(data: &[N]) -> Self {
        Self::new(
            Vector3::new(data[0], data[1], data[2]),
            Vector3::new(data[3], data[4], data[5]),
        )
    }

    /// Compute the velocity of a point that is located at the coordinates `shift` relative to the point having `self` as velocity.
    #[inline]
    pub fn shift(&self, shift: &Vector3<N>) -> Self {
        Self::new(self.linear + self.angular.cross(&shift), self.angular)
    }

    /// Transform each component of `self` by `iso`.
    #[inline]
    pub fn transformed(&self, iso: &Isometry3<N>) -> Self {
        Self::new(iso * self.linear, iso * self.angular)
    }

    /// Rotate each component of `self` by `rot`.
    #[inline]
    pub fn rotated(&self, rot: &UnitQuaternion<N>) -> Self {
        Self::new(rot * self.linear, rot * self.angular)
    }
}

impl<N: RealField> Add<Velocity3<N>> for Velocity3<N> {
    type Output = Self;

    #[inline]
    fn add(self, rhs: Self) -> Self {
        Velocity3::new(self.linear + rhs.linear, self.angular + rhs.angular)
    }
}

impl<N: RealField> AddAssign<Velocity3<N>> for Velocity3<N> {
    #[inline]
    fn add_assign(&mut self, rhs: Self) {
        self.linear += rhs.linear;
        self.angular += rhs.angular;
    }
}

impl<N: RealField> Sub<Velocity3<N>> for Velocity3<N> {
    type Output = Self;

    #[inline]
    fn sub(self, rhs: Self) -> Self {
        Velocity3::new(self.linear - rhs.linear, self.angular - rhs.angular)
    }
}

impl<N: RealField> SubAssign<Velocity3<N>> for Velocity3<N> {
    #[inline]
    fn sub_assign(&mut self, rhs: Self) {
        self.linear -= rhs.linear;
        self.angular -= rhs.angular;
    }
}

impl<N: RealField> Mul<N> for Velocity3<N> {
    type Output = Self;

    #[inline]
    fn mul(self, rhs: N) -> Self {
        Velocity3::new(self.linear * rhs, self.angular * rhs)
    }
}
