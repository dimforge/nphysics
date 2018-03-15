use std::ops::{Deref, DerefMut};

use na::{DVectorSlice, DVectorSliceMut, Real};

use object::{BodyHandle, Multibody};
use joint::Joint;
use math::{Force, Inertia, Isometry, Point, Vector, Velocity};

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

    pub fn ground() -> Self {
        MultibodyLinkId {
            internal_id: usize::max_value(),
        }
    }

    pub fn is_root(&self) -> bool {
        self.internal_id == 0
    }

    pub fn is_ground(&self) -> bool {
        self.internal_id == usize::max_value()
    }
}

pub struct MultibodyLink<N: Real> {
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

    // Contants.
    pub local_inertia: Inertia<N>,
    pub inertia: Inertia<N>,
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
    ) -> Self {
        let is_leaf = true;
        let velocity = Velocity::zero();
        let velocity_dot_wrt_joint = Velocity::zero();
        let velocity_wrt_joint = Velocity::zero();
        let external_forces = Force::zero();
        let inertia = local_inertia.transformed(&local_to_world);

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
            inertia,
        }
    }

    pub fn center_of_mass(&self) -> Point<N> {
        Point::from_coordinates(self.local_to_world.translation.vector)
    }
}

pub struct MultibodyLinkMut<'a, N: Real> {
    id: MultibodyLinkId,
    multibody: &'a mut Multibody<N>,
}

impl<'a, N: Real> MultibodyLinkMut<'a, N> {
    pub fn new(id: MultibodyLinkId, multibody: &'a mut Multibody<N>) -> Self {
        MultibodyLinkMut { id, multibody }
    }

    #[inline]
    pub fn handle(&self) -> BodyHandle {
        self.multibody.rbs()[self.id.internal_id].handle
    }

    #[inline]
    pub(crate) fn id(&self) -> MultibodyLinkId {
        self.id
    }

    #[inline]
    pub fn joint_mut(&mut self) -> &mut Joint<N> {
        &mut *self.multibody.rbs_mut()[self.id.internal_id].dof
    }

    // FIXME: add methods to modify velocities, forces, damping, etc.

    #[inline]
    pub fn joint_velocity_mut(&mut self) -> DVectorSliceMut<N> {
        let ndofs = self.multibody.rbs()[self.id.internal_id].dof.ndofs();
        let id = self.multibody.rbs()[self.id.internal_id].assembly_id;
        let vels = self.multibody.generalized_velocity_slice_mut();
        DVectorSliceMut::new(&mut vels[id..id + ndofs], ndofs)
    }
}

pub struct MultibodyLinkRef<'a, N: Real> {
    id: MultibodyLinkId,
    multibody: &'a Multibody<N>,
    link: &'a MultibodyLink<N>,
}

impl<'a, N: Real> MultibodyLinkRef<'a, N> {
    #[inline]
    pub fn new(id: MultibodyLinkId, multibody: &'a Multibody<N>) -> Self {
        let link = &multibody.rbs()[id.internal_id];
        MultibodyLinkRef {
            id,
            multibody,
            link,
        }
    }

    #[inline]
    pub fn handle(&self) -> BodyHandle {
        self.link.handle
    }

    #[inline]
    pub fn joint(&self) -> &Joint<N> {
        &*self.link.dof
    }

    #[inline]
    pub fn center_of_mass(&self) -> Point<N> {
        self.link.center_of_mass()
    }

    #[inline]
    pub fn velocity(&self) -> &Velocity<N> {
        &self.link.velocity
    }

    #[inline]
    pub fn joint_velocity(&self) -> DVectorSlice<N> {
        let vels = self.multibody.generalized_velocity_slice();
        let ndofs = self.link.dof.ndofs();
        DVectorSlice::new(
            &vels[self.link.assembly_id..self.link.assembly_id + ndofs],
            ndofs,
        )
    }

    #[inline]
    pub fn id(&self) -> MultibodyLinkId {
        self.id
    }

    #[inline]
    pub fn assembly_id(&self) -> usize {
        self.link.assembly_id
    }

    #[inline]
    pub fn impulse_id(&self) -> usize {
        self.link.impulse_id
    }

    #[inline]
    pub fn multibody(&self) -> &Multibody<N> {
        self.multibody
    }

    pub fn inv_mass_mul_unit_joint_force(&self, dof_id: usize, force: N, out: &mut [N]) {
        self.multibody
            .inv_mass_mul_unit_joint_force(self.id, dof_id, force, out)
    }

    pub fn inv_mass_mul_joint_force(&self, force: DVectorSlice<N>, out: &mut [N]) {
        self.multibody.inv_mass_mul_joint_force(self.id, force, out)
    }

    #[inline]
    pub fn position(&self) -> Isometry<N> {
        self.link.local_to_world
    }

    #[inline]
    pub fn body_jacobian_mul_force(&self, force: &Force<N>, out: &mut [N]) {
        self.multibody.body_jacobian_mul_force(self.id, force, out)
    }

    #[inline]
    pub fn inv_mass_mul_generalized_forces(&self, out: &mut [N]) {
        self.multibody.inv_mass_mul_generalized_forces(out)
    }

    #[inline]
    pub fn inv_mass_mul_force(&self, force: &Force<N>, out: &mut [N]) {
        self.multibody.inv_mass_mul_force(self.id, force, out)
    }
}

// FIXME: keep this even if we already have the Index2 traits?
pub struct MultibodyLinkVec<N: Real>(pub Vec<MultibodyLink<N>>);

impl<N: Real> MultibodyLinkVec<N> {
    #[inline]
    pub fn get_with_parent(&self, i: usize) -> (&MultibodyLink<N>, &MultibodyLink<N>) {
        assert!(
            i != 0,
            "The parent of this body is not part of the same multibody."
        );
        let rb = &self[i];
        let parent_rb = &self[rb.parent.internal_id];

        (rb, parent_rb)
    }

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
