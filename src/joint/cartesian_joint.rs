use na::{self, DVectorSliceMut, RealField};

use crate::joint::Joint;

use crate::math::{Isometry, JacobianSliceMut, Translation, Vector, Velocity, DIM};
use crate::solver::IntegrationParameters;

/// A joint that allows only all the translational degrees of freedom between two multibody links.
#[derive(Copy, Clone, Debug)]
pub struct CartesianJoint<N: RealField> {
    position: Vector<N>,
}

impl<N: RealField> CartesianJoint<N> {
    /// Create a cartesian joint with an initial position given by `position`.
    pub fn new(position: Vector<N>) -> Self {
        CartesianJoint { position }
    }
}

impl<N: RealField> Joint<N> for CartesianJoint<N> {
    #[inline]
    fn ndofs(&self) -> usize {
        DIM
    }

    fn body_to_parent(&self, parent_shift: &Vector<N>, body_shift: &Vector<N>) -> Isometry<N> {
        let t = Translation::from(parent_shift - body_shift + self.position);
        Isometry::from_parts(t, na::one())
    }

    fn update_jacobians(&mut self, _: &Vector<N>, _: &[N]) {}

    fn jacobian(&self, _: &Isometry<N>, out: &mut JacobianSliceMut<N>) {
        out.fill_diagonal(N::one())
    }

    fn jacobian_dot(&self, _: &Isometry<N>, _: &mut JacobianSliceMut<N>) {}

    fn jacobian_dot_veldiff_mul_coordinates(
        &self,
        _: &Isometry<N>,
        _: &[N],
        _: &mut JacobianSliceMut<N>,
    ) {
    }

    fn jacobian_mul_coordinates(&self, vels: &[N]) -> Velocity<N> {
        Velocity::from_vectors(Vector::from_row_slice(&vels[..DIM]), na::zero())
    }

    fn jacobian_dot_mul_coordinates(&self, _: &[N]) -> Velocity<N> {
        Velocity::zero()
    }

    fn default_damping(&self, _: &mut DVectorSliceMut<N>) {}

    fn integrate(&mut self, parameters: &IntegrationParameters<N>, vels: &[N]) {
        self.position += Vector::from_row_slice(&vels[..DIM]) * parameters.dt();
    }

    fn apply_displacement(&mut self, disp: &[N]) {
        self.position += Vector::from_row_slice(&disp[..DIM]);
    }

    #[inline]
    fn clone(&self) -> Box<dyn Joint<N>> {
        Box::new(*self)
    }
}
