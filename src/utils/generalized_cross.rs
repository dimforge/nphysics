use alga::general::Real;
use na::{Vector1, Vector2, Vector3, Matrix3, RowVector2};

/// This is a non-standard generalization of the cross product design exclusively to group the
/// 3D cross product and the 2D perpendicular product behind the same interface.
pub trait GeneralizedCross {
    /// The result type of the this (non-standard) generalized cross product.
    type CrossVector;
    /// The matrix representation of this (non-standard) generalized cross product.
    type CrossMatrix;

    /// Computes this (non-standard) generalized cross product.
    fn gcross(&self, rhs: &Self) -> Self::CrossVector;

    /// Computes the matrix represenattion of this (non-standard) generalized cross product.
    fn gcross_matrix(&self) -> Self::CrossMatrix;
}

impl<N: Real> GeneralizedCross for Vector2<N> {
    type CrossVector = Vector1<N>;
    type CrossMatrix = RowVector2<N>;

    #[inline]
    fn gcross(&self, rhs: &Vector2<N>) -> Vector1<N> {
        Vector1::new(self.x * rhs.y - self.y * rhs.x)
    }

    #[inline]
    fn gcross_matrix(&self) -> RowVector2<N> {
        RowVector2::new(-self.y, self.x)
    }
}

impl<N: Real> GeneralizedCross for Vector3<N> {
    type CrossVector = Vector3<N>;
    type CrossMatrix = Matrix3<N>;

    #[inline]
    fn gcross(&self, rhs: &Vector3<N>) -> Vector3<N> {
        self.cross(rhs)
    }

    #[inline]
    fn gcross_matrix(&self) -> Matrix3<N> {
        Matrix3::new(
            N::zero(), -self.z,   self.y,
            self.z,    N::zero(), -self.x,
            -self.y,   self.x,    N::zero()
        )
    }
}
