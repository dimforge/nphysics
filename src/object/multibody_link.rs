use std::ops::{Deref, DerefMut};

use na::{DVectorSlice, DVectorSliceMut, Real};

use joint::Joint;
use math::{Force, Inertia, Isometry, Point, Vector, Velocity};
use object::{BodyHandle, Multibody};

/// The identifier of a multibody link.
#[derive(Copy, Clone, Hash, Debug, PartialEq, Eq)]
pub struct MultibodyLinkId {
    pub(crate) internal_id: usize,
}

impl MultibodyLinkId {
    pub(crate) fn new(internal_id: usize) -> Self {
        MultibodyLinkId {
            internal_id: internal_id,
        }
    }

    /// The multibody link identifying the ground.
    pub fn ground() -> Self {
        MultibodyLinkId {
            internal_id: usize::max_value(),
        }
    }

    /// Return `true` if this link is at the root of the multibody.
    pub fn is_root(&self) -> bool {
        self.internal_id == 0
    }

    /// Return `true` if this link identifies the ground.
    pub fn is_ground(&self) -> bool {
        self.internal_id == usize::max_value()
    }
}

pub(crate) struct MultibodyLink<N: Real> {
    pub handle: BodyHandle,
    pub assembly_id: usize,
    pub impulse_id: usize,
    pub is_leaf: bool,

    // XXX: rename to "joint".
    // (And rename the full-coordinates joint contsraints JointConstraint).
    pub parent: MultibodyLinkId,
    pub dof: Box<Joint<N>>,
    pub parent_shift: Vector<N>,
    pub body_shift: Vector<N>,

    // Change at each time step.
    pub parent_to_world: Isometry<N>,
    pub local_to_world: Isometry<N>,
    pub local_to_parent: Isometry<N>,
    // FIXME: put this on a workspace buffer instead ?
    pub velocity_dot_wrt_joint: Velocity<N>, // J' q' in world space. FIXME: what could be a better name ?
    pub velocity_wrt_joint: Velocity<N>,     // J  q' in world space.
    pub velocity: Velocity<N>,
    pub external_forces: Force<N>,
    pub inertia: Inertia<N>,
    pub com: Point<N>,

    pub local_inertia: Inertia<N>,
    pub local_com: Point<N>,
    // TODO: User-defined data
    // user_data:       T
}

impl<N: Real> MultibodyLink<N> {
    pub fn new(
        handle: BodyHandle,
        assembly_id: usize,
        impulse_id: usize,
        parent: MultibodyLinkId,
        dof: Box<Joint<N>>,
        parent_shift: Vector<N>,
        body_shift: Vector<N>,
        parent_to_world: Isometry<N>,
        local_to_world: Isometry<N>,
        local_to_parent: Isometry<N>,
        local_inertia: Inertia<N>,
        local_com: Point<N>,
    ) -> Self {
        let is_leaf = true;
        let velocity = Velocity::zero();
        let velocity_dot_wrt_joint = Velocity::zero();
        let velocity_wrt_joint = Velocity::zero();
        let external_forces = Force::zero();
        let inertia = local_inertia.transformed(&local_to_world);
        let com = local_to_world * local_com;

        MultibodyLink {
            handle,
            assembly_id,
            impulse_id,
            is_leaf,
            parent,
            dof,
            parent_shift,
            body_shift,
            parent_to_world,
            local_to_world,
            local_to_parent,
            velocity_dot_wrt_joint,
            velocity_wrt_joint,
            velocity,
            external_forces,
            local_inertia,
            local_com,
            inertia,
            com,
        }
    }

    pub fn center_of_mass(&self) -> Point<N> {
        self.com
    }
}

/// Mutable reference to a multibody link.
pub struct MultibodyLinkMut<'a, N: Real> {
    id: MultibodyLinkId,
    multibody: &'a mut Multibody<N>,
}

impl<'a, N: Real> MultibodyLinkMut<'a, N> {
    /// Creates a new mutable reference of a multibody link.
    pub fn new(id: MultibodyLinkId, multibody: &'a mut Multibody<N>) -> Self {
        MultibodyLinkMut { id, multibody }
    }

    /// The handle of this link.
    #[inline]
    pub fn handle(&self) -> BodyHandle {
        self.multibody.rbs()[self.id.internal_id].handle
    }

    /// Retrieve an immutable reference to this link.
    pub fn as_ref(&self) -> MultibodyLinkRef<'_, N> {
        MultibodyLinkRef::new(self.id, &*self.multibody)
    }

    #[inline]
    pub(crate) fn id(&self) -> MultibodyLinkId {
        self.id
    }

    /// Mutable reference to the joint attaching this link to its parent.
    #[inline]
    pub fn joint_mut(&mut self) -> &mut Joint<N> {
        &mut *self.multibody.rbs_mut()[self.id.internal_id].dof
    }

    /// Apply a force to this link.
    #[inline]
    pub fn apply_force(&mut self, force: &Force<N>) {
        let rb = &mut self.multibody.rbs_mut()[self.id.internal_id];
        rb.external_forces.linear += force.linear;
        rb.external_forces.angular += force.angular;
    }

    // FIXME: add methods to modify velocities, forces, damping, etc.

    /// Retriev the mutable generalized velocities of this link.
    #[inline]
    pub fn joint_velocity_mut(&mut self) -> DVectorSliceMut<N> {
        let ndofs = self.multibody.rbs()[self.id.internal_id].dof.ndofs();
        let id = self.multibody.rbs()[self.id.internal_id].assembly_id;
        let vels = self.multibody.generalized_velocity_slice_mut();
        DVectorSliceMut::from_slice(&mut vels[id..id + ndofs], ndofs)
    }
}

/// A reference to a multibody link.
pub struct MultibodyLinkRef<'a, N: Real> {
    id: MultibodyLinkId,
    multibody: &'a Multibody<N>,
    link: &'a MultibodyLink<N>,
}

impl<'a, N: Real> MultibodyLinkRef<'a, N> {
    /// Create a new reference to a multibody link.
    #[inline]
    pub fn new(id: MultibodyLinkId, multibody: &'a Multibody<N>) -> Self {
        let link = &multibody.rbs()[id.internal_id];
        MultibodyLinkRef {
            id,
            multibody,
            link,
        }
    }

    /// The handle of this link.
    #[inline]
    pub fn handle(&self) -> BodyHandle {
        self.link.handle
    }

    /// Reference to the joint attaching this link to its parent.
    #[inline]
    pub fn joint(&self) -> &Joint<N> {
        &*self.link.dof
    }

    /// Center of mass of this link wrt. the ground.
    #[inline]
    pub fn center_of_mass(&self) -> Point<N> {
        self.link.center_of_mass()
    }

    /// Velocity at the center of mass of this link.
    #[inline]
    pub fn velocity(&self) -> &Velocity<N> {
        &self.link.velocity
    }

    /// Generalized velocities of this link.
    #[inline]
    pub fn joint_velocity(&self) -> DVectorSlice<N> {
        let vels = self.multibody.generalized_velocity_slice();
        let ndofs = self.link.dof.ndofs();
        DVectorSlice::from_slice(
            &vels[self.link.assembly_id..self.link.assembly_id + ndofs],
            ndofs,
        )
    }

    /// The identifier of this link.
    #[inline]
    pub fn id(&self) -> MultibodyLinkId {
        self.id
    }

    /// The dynamic assembly identifier of this link.
    #[inline]
    pub fn assembly_id(&self) -> usize {
        self.link.assembly_id
    }

    /// The impulse cache identifier of this link.
    #[inline]
    pub fn impulse_id(&self) -> usize {
        self.link.impulse_id
    }

    /// Reference to the multibody containing this link.
    #[inline]
    pub fn multibody(&self) -> &Multibody<N> {
        self.multibody
    }

    /// Convert a generalized force applied to this link degrees of freedom into generalized accelerations.
    ///
    /// The joint attaching this link to its parent is assumed to be a unit joint.
    pub fn inv_mass_mul_unit_joint_force(&self, dof_id: usize, force: N, out: &mut [N]) {
        self.multibody
            .inv_mass_mul_unit_joint_force(self.id, dof_id, force, out)
    }

    /// Convert a generalized force applied to this link degrees of freedom into generalized accelerations.
    pub fn inv_mass_mul_joint_force(&self, force: DVectorSlice<N>, out: &mut [N]) {
        self.multibody.inv_mass_mul_joint_force(self.id, force, out)
    }

    /// The position of the center of mass of this link wrt. the ground.
    #[inline]
    pub fn position(&self) -> Isometry<N> {
        self.link.local_to_world
    }

    /// Convert a force applied to this link center of mass into generalized force.
    #[inline]
    pub fn body_jacobian_mul_force(&self, force: &Force<N>, out: &mut [N]) {
        self.multibody.body_jacobian_mul_force(self.id, force, out)
    }

    /// Convert generalized forces applied to the multibody containing this link into generalized accelerations.
    #[inline]
    pub fn inv_mass_mul_generalized_forces(&self, out: &mut [N]) {
        self.multibody.inv_mass_mul_generalized_forces(out)
    }

    /// Convert a force applied to this link's center of mass into generalized accelerations.
    #[inline]
    pub fn inv_mass_mul_force(&self, force: &Force<N>, out: &mut [N]) {
        self.multibody.inv_mass_mul_force(self.id, force, out)
    }

    /// The local inertia of this link.
    #[inline]
    pub fn local_inertia(&self) -> &Inertia<N> {
        &self.link.local_inertia
    }

    /// The world-space inertia of this link.
    #[inline]
    pub fn inertia(&self) -> &Inertia<N> {
        &self.link.inertia
    }
}

// FIXME: keep this even if we already have the Index2 traits?
pub(crate) struct MultibodyLinkVec<N: Real>(pub Vec<MultibodyLink<N>>);

impl<N: Real> MultibodyLinkVec<N> {
    #[inline]
    // pub fn get_with_parent(&self, i: usize) -> (&MultibodyLink<N>, &MultibodyLink<N>) {
    //     assert!(
    //         i != 0,
    //         "The parent of this body is not part of the same multibody."
    //     );
    //     let rb = &self[i];
    //     let parent_rb = &self[rb.parent.internal_id];

    //     (rb, parent_rb)
    // }
    #[inline]
    pub fn get_mut_with_parent(&mut self, i: usize) -> (&mut MultibodyLink<N>, &MultibodyLink<N>) {
        let parent_id = self[i].parent.internal_id;

        assert!(
            parent_id != i,
            "Internal error: circular rigid body dependency."
        );
        assert!(parent_id < self.len(), "Invalid parent index.");

        unsafe {
            let rb = &mut *(self.get_unchecked_mut(i) as *mut _);
            let parent_rb = &*(self.get_unchecked(parent_id) as *const _);
            (rb, parent_rb)
        }
    }

    #[inline]
    pub fn unwrap(self) -> Vec<MultibodyLink<N>> {
        self.0
    }
}

impl<N: Real> Deref for MultibodyLinkVec<N> {
    type Target = Vec<MultibodyLink<N>>;

    #[inline]
    fn deref(&self) -> &Vec<MultibodyLink<N>> {
        let MultibodyLinkVec(ref me) = *self;
        me
    }
}

impl<N: Real> DerefMut for MultibodyLinkVec<N> {
    #[inline]
    fn deref_mut(&mut self) -> &mut Vec<MultibodyLink<N>> {
        let MultibodyLinkVec(ref mut me) = *self;
        me
    }
}
