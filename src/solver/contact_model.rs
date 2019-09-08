#![allow(missing_docs)]

use na::{DVector, RealField};
use ncollide::query::ContactId;

use crate::detection::ColliderContactManifold;
use crate::object::{BodySet, ColliderHandle, BodyHandle};
use crate::material::MaterialsCoefficientsTable;
use crate::solver::{ConstraintSet, IntegrationParameters};

/// The modeling of a contact.
pub trait ContactModel<N: RealField, Handle: BodyHandle, CollHandle: ColliderHandle>: Send + Sync {
    /// Maximum number of velocity constraint to be generated for each contact.
    fn num_velocity_constraints(&self, manifold: &ColliderContactManifold<N, Handle, CollHandle>) -> usize;
    /// Generate all constraints for the given contact manifolds.
    fn constraints<Bodies: BodySet<N, Handle=Handle>>(
        &mut self,
        parameters: &IntegrationParameters<N>,
        material_coefficients: &MaterialsCoefficientsTable<N>,
        bodies: &Bodies,
        ext_vels: &DVector<N>,
        manifolds: &[ColliderContactManifold<N, Handle, CollHandle>],
        ground_j_id: &mut usize,
        j_id: &mut usize,
        jacobians: &mut [N],
        constraints: &mut ConstraintSet<N, Handle, CollHandle, ContactId>,
    );

    /// Stores all the impulses found by the solver into a cache for warmstarting.
    fn cache_impulses(&mut self, constraints: &ConstraintSet<N, Handle, CollHandle, ContactId>);
}
