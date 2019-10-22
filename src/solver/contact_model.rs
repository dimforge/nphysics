#![allow(missing_docs)]

use downcast_rs::Downcast;
use na::{DVector, RealField};
use ncollide::query::ContactId;

use crate::detection::ColliderContactManifold;
use crate::material::MaterialsCoefficientsTable;
use crate::object::{BodyHandle, BodySet, ColliderHandle};
use crate::solver::{ConstraintSet, IntegrationParameters};

/// The modeling of a contact.
pub trait ContactModel<N: RealField, Handle: BodyHandle, CollHandle: ColliderHandle>:
    Downcast + Send + Sync
{
    /// Maximum number of velocity constraint to be generated for each contact.
    fn num_velocity_constraints(
        &self,
        manifold: &ColliderContactManifold<N, Handle, CollHandle>,
    ) -> usize;
    /// Generate all constraints for the given contact manifolds.
    fn constraints(
        &mut self,
        parameters: &IntegrationParameters<N>,
        material_coefficients: &MaterialsCoefficientsTable<N>,
        bodies: &dyn BodySet<N, Handle = Handle>,
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

impl_downcast!(ContactModel<N, Handle, CollHandle> where N: RealField, Handle: BodyHandle, CollHandle: ColliderHandle);
