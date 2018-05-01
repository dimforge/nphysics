use downcast::Any;

use na::Real;

use math::{Isometry, JacobianSliceMut, Vector, Velocity};
use object::MultibodyLinkRef;
use solver::{ConstraintSet, GenericNonlinearConstraint, IntegrationParameters,
             UnilateralGroundConstraint};

pub trait Joint<N: Real>: Any + Send + Sync {
    fn ndofs(&self) -> usize;
    fn body_to_parent(&self, parent_shift: &Vector<N>, body_shift: &Vector<N>) -> Isometry<N>;
    fn update_jacobians(&mut self, body_shift: &Vector<N>, vels: &[N]);
    fn integrate(&mut self, params: &IntegrationParameters<N>, vels: &[N]);
    fn apply_displacement(&mut self, disp: &[N]);

    // FIXME: rename those "copy_jacobian_to" ?
    fn jacobian(&self, transform: &Isometry<N>, out: &mut JacobianSliceMut<N>);
    fn jacobian_dot(&self, transform: &Isometry<N>, out: &mut JacobianSliceMut<N>);
    fn jacobian_dot_veldiff_mul_coordinates(
        &self,
        transform: &Isometry<N>,
        vels: &[N],
        out: &mut JacobianSliceMut<N>,
    );

    fn jacobian_mul_coordinates(&self, vels: &[N]) -> Velocity<N>;
    fn jacobian_dot_mul_coordinates(&self, vels: &[N]) -> Velocity<N>;

    /// The maximum number of impulses needed by this joints for
    /// its constraints.
    fn nimpulses(&self) -> usize {
        // FIXME: keep this?
        self.ndofs() * 3
    }

    fn num_velocity_constraints(&self) -> usize {
        0
    }

    fn velocity_constraints(
        &self,
        _params: &IntegrationParameters<N>,
        _link: &MultibodyLinkRef<N>,
        _assembly_id: usize,
        _dof_id: usize,
        _ext_vels: &[N],
        _ground_j_id: &mut usize,
        _jacobians: &mut [N],
        _velocity_constraints: &mut ConstraintSet<N>,
    ) {
    }

    fn num_position_constraints(&self) -> usize {
        0
    }

    fn position_constraint(
        &self,
        _i: usize,
        _link: &MultibodyLinkRef<N>,
        _dof_id: usize,
        _jacobians: &mut [N],
    ) -> Option<GenericNonlinearConstraint<N>> {
        None
    }
}

downcast!(<N> Joint<N> where N: Real);
