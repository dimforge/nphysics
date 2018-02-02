use na::Real;

use joint::Joint;
use solver::IntegrationParameters;
use math::{Isometry, JacobianSliceMut, Vector, Velocity, SPATIAL_DIM};

#[derive(Copy, Clone, Debug)]
pub struct FreeJoint<N: Real> {
    position: Isometry<N>,
}

impl<N: Real> FreeJoint<N> {
    pub fn new(position: Isometry<N>) -> Self {
        FreeJoint { position }
    }
}

impl<N: Real> Joint<N> for FreeJoint<N> {
    fn ndofs(&self) -> usize {
        SPATIAL_DIM
    }

    fn body_to_parent(&self, _: &Vector<N>, _: &Vector<N>) -> Isometry<N> {
        self.position
    }

    fn update_jacobians(&mut self, _: &Vector<N>, _: &[N]) {}

    fn jacobian(&self, _: &Isometry<N>, out: &mut JacobianSliceMut<N>) {
        out.fill_diagonal(N::one());
    }

    fn jacobian_dot(&self, _: &Isometry<N>, _: &mut JacobianSliceMut<N>) {}

    fn jacobian_dot_veldiff_mul_coordinates(
        &self,
        _: &Isometry<N>,
        _: &[N],
        _: &mut JacobianSliceMut<N>,
    ) {
    }

    fn apply_displacement(&mut self, params: &IntegrationParameters<N>, vels: &[N]) {
        let vel = Velocity::from_slice(vels);
        let disp = Isometry::new(vel.linear * params.dt, vel.angular * params.dt);
        self.position = Isometry::from_parts(
            disp.translation * self.position.translation,
            disp.rotation * self.position.rotation,
        )
    }

    fn jacobian_mul_coordinates(&self, vels: &[N]) -> Velocity<N> {
        Velocity::from_slice(vels)
    }

    fn jacobian_dot_mul_coordinates(&self, _: &[N]) -> Velocity<N> {
        // The jacobian derivative is zero.
        Velocity::zero()
    }
}
