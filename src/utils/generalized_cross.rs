use na::RealField;
use na::{Matrix2, Matrix3, RowVector2, Vector1, Vector2, Vector3};

/// This is a non-standard generalization of the cross product design exclusively to group the
/// 3D cross product and the 2D perpendicular product behind the same interface.
pub trait GeneralizedCross {
    /// The right-hand-side of this cross product.
    type Rhs;
    /// The result type of the this (non-standard) generalized cross product.
    type CrossVector;
    /// The matrix representation of this (non-standard) generalized cross product.
    type CrossMatrix;
    /// The transposed matrix representation of this (non-standard) generalized cross product.
    type CrossMatrixTr;

    /// Computes this (non-standard) generalized cross product.
    fn gcross(&self, rhs: &Self::Rhs) -> Self::CrossVector;

    /// Computes the matrix represenattion of this (non-standard) generalized cross product.
    fn gcross_matrix(&self) -> Self::CrossMatrix;

    /// Computes the transposed matrix represenattion of this (non-standard) generalized cross product.
    fn gcross_matrix_tr(&self) -> Self::CrossMatrixTr;
}

impl<N: RealField> GeneralizedCross for Vector1<N> {
    type Rhs = Vector2<N>;
    type CrossVector = Vector2<N>;
    type CrossMatrix = Matrix2<N>;
    type CrossMatrixTr = Matrix2<N>;

    #[inline]
    fn gcross(&self, rhs: &Vector2<N>) -> Vector2<N> {
        Vector2::new(-rhs.y * self.x, rhs.x * self.x)
    }

    #[inline]
    fn gcross_matrix(&self) -> Matrix2<N> {
        Matrix2::new(N::zero(), -self.x, self.x, N::zero())
    }

    #[inline]
    fn gcross_matrix_tr(&self) -> Matrix2<N> {
        Matrix2::new(N::zero(), self.x, -self.x, N::zero())
    }
}

impl<N: RealField> GeneralizedCross for Vector2<N> {
    type Rhs = Vector2<N>;
    type CrossVector = Vector1<N>;
    type CrossMatrix = RowVector2<N>;
    type CrossMatrixTr = Vector2<N>;

    #[inline]
    fn gcross(&self, rhs: &Vector2<N>) -> Vector1<N> {
        Vector1::new(self.x * rhs.y - self.y * rhs.x)
    }

    #[inline]
    fn gcross_matrix(&self) -> RowVector2<N> {
        RowVector2::new(-self.y, self.x)
    }

    #[inline]
    fn gcross_matrix_tr(&self) -> Vector2<N> {
        Vector2::new(-self.y, self.x)
    }
}

impl<N: RealField> GeneralizedCross for Vector3<N> {
    type Rhs = Vector3<N>;
    type CrossVector = Vector3<N>;
    type CrossMatrix = Matrix3<N>;
    type CrossMatrixTr = Matrix3<N>;

    #[inline]
    fn gcross(&self, rhs: &Vector3<N>) -> Vector3<N> {
        self.cross(rhs)
    }

    #[inline]
    fn gcross_matrix(&self) -> Matrix3<N> {
        self.cross_matrix()
    }

    #[inline]
    fn gcross_matrix_tr(&self) -> Matrix3<N> {
        Matrix3::new(
            N::zero(),
            self.z,
            -self.y,
            -self.z,
            N::zero(),
            self.x,
            self.y,
            -self.x,
            N::zero(),
        )
    }
}
