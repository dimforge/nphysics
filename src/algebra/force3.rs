use std::mem;
use std::ops::{Neg, Add, AddAssign, Mul, Sub, SubAssign};
use na::{self, Isometry3, Point3, Real, U6, Vector, Vector3, Vector6};
use na::storage::Storage;

#[derive(Copy, Clone, Debug)]
pub struct Force3<N: Real> {
    pub linear: Vector3<N>,
    pub angular: Vector3<N>,
}

impl<N: Real> Force3<N> {
    #[inline]
    pub fn new(linear: Vector3<N>, angular: Vector3<N>) -> Self {
        Force3 { linear, angular }
    }

    #[inline]
    pub fn zero() -> Self {
        Self::new(na::zero(), na::zero())
    }

    #[inline]
    pub fn from_slice(data: &[N]) -> Self {
        Self::new(
            Vector3::new(data[0], data[1], data[2]),
            Vector3::new(data[3], data[4], data[5]),
        )
    }

    #[inline]
    pub fn from_vector<S: Storage<N, U6>>(data: &Vector<N, U6, S>) -> Self {
        Self::new(
            Vector3::new(data[0], data[1], data[2]),
            Vector3::new(data[3], data[4], data[5]),
        )
    }

    #[inline]
    pub fn torque(torque: Vector3<N>) -> Self {
        Self::new(na::zero(), torque)
    }

    #[inline]
    pub fn torque_from_vector(torque: Vector3<N>) -> Self {
        Self::new(na::zero(), torque)
    }

    #[inline]
    pub fn torque_at_point(torque: Vector3<N>, point: &Point3<N>) -> Self {
        Self::new(-torque.cross(&point.coords), torque)
    }

    #[inline]
    pub fn torque_from_vector_at_point(torque: Vector3<N>, point: &Point3<N>) -> Self {
        Self::torque_at_point(torque, point)
    }
    
    #[inline]
    pub fn linear(linear: Vector3<N>) -> Self {
        Self::new(linear, na::zero())
    }

    #[inline]
    pub fn linear_at_point(linear: Vector3<N>, point: &Point3<N>) -> Self {
        Self::new(linear, point.coords.cross(&linear))
    }

    /// This twist seen as a slice.
    #[inline]
    pub fn as_slice(&self) -> &[N] {
        self.as_vector().as_slice()
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
    pub fn to_transform(&self) -> Isometry3<N> {
        Isometry3::new(self.linear, self.angular)
    }

    // #[inline]
    // pub fn frame_velocity(&self, frame: &Isometry3<N>) -> Self {
    //     Self::new(self.linear + self.angular.cross(&frame.translation.vector), self.angular)
    // }

    // // XXX: not a good name
    // #[inline]
    // pub fn shift(&self, shift: &Vector3<N>) -> Self {
    //     Self::new(self.linear + self.angular.cross(&shift), self.angular)
    // }

    // #[inline]
    // pub fn rotated(&self, rot: &Rotation<N>) -> Self {
    //     Self::new(rot * self.linear, rot * self.angular)
    // }

    // #[inline]
    // pub fn transformed(&self, iso: &Isometry3<N>) -> Self {
    //     Self::new(iso * self.linear, iso * self.angular)
    // }
}

impl<N: Real> Add<Force3<N>> for Force3<N> {
    type Output = Self;

    #[inline]
    fn add(self, rhs: Self) -> Self {
        Force3::new(self.linear + rhs.linear, self.angular + rhs.angular)
    }
}

impl<N: Real> AddAssign<Force3<N>> for Force3<N> {
    #[inline]
    fn add_assign(&mut self, rhs: Self) {
        self.linear += rhs.linear;
        self.angular += rhs.angular;
    }
}

impl<N: Real> Sub<Force3<N>> for Force3<N> {
    type Output = Self;

    #[inline]
    fn sub(self, rhs: Self) -> Self {
        Force3::new(self.linear - rhs.linear, self.angular - rhs.angular)
    }
}

impl<N: Real> SubAssign<Force3<N>> for Force3<N> {
    #[inline]
    fn sub_assign(&mut self, rhs: Self) {
        self.linear -= rhs.linear;
        self.angular -= rhs.angular;
    }
}

impl<N: Real> Mul<N> for Force3<N> {
    type Output = Self;

    #[inline]
    fn mul(self, rhs: N) -> Self {
        Force3::new(self.linear * rhs, self.angular * rhs)
    }
}

impl<N: Real> Neg for Force3<N> {
    type Output = Self;

    #[inline]
    fn neg(self) -> Self {
        Force3::new(-self.linear, -self.angular)
    }
}