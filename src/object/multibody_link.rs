use std::ops::{Deref, DerefMut};

use na::Real;

use joint::Joint;
use math::{Force, Inertia, Isometry, Point, Vector, Velocity};
use object::{BodyPartHandle, Body, BodyPart, BodyHandle};

/// One link of a multibody.
pub struct MultibodyLink<N: Real> {
    // FIXME: make all those private.
    pub(crate) multibody_handle: Option<BodyHandle>,
    pub(crate) part_id: usize,
    pub(crate) internal_id: usize,
    pub(crate) assembly_id: usize,
    pub(crate) impulse_id: usize,
    pub(crate) is_leaf: bool,

    // XXX: rename to "joint".
    // (And rename the full-coordinates joint constraints JointConstraint).
    pub(crate) parent: BodyPartHandle,
    pub(crate) parent_internal_id: usize,
    pub(crate) dof: Box<Joint<N>>,
    pub(crate) parent_shift: Vector<N>,
    pub(crate) body_shift: Vector<N>,

    // Change at each time step.
    pub(crate) parent_to_world: Isometry<N>,
    pub(crate) local_to_world: Isometry<N>,
    pub(crate) local_to_parent: Isometry<N>,
    // FIXME: put this on a workspace buffer instead ?
    pub(crate) velocity_dot_wrt_joint: Velocity<N>,
    // J' q' in world space. FIXME: what could be a better name ?
    pub(crate) velocity_wrt_joint: Velocity<N>,
    // J  q' in world space.
    pub(crate) velocity: Velocity<N>,
    pub(crate) external_forces: Force<N>,
    pub(crate) inertia: Inertia<N>,
    pub(crate) com: Point<N>,

    pub(crate) local_inertia: Inertia<N>,
    pub(crate) local_com: Point<N>,
    // TODO: User-defined data
    // user_data:       T
}

impl<N: Real> MultibodyLink<N> {
    /// Creates a new multibody link.
    pub fn new(
        part_id: usize,
        internal_id: usize,
        assembly_id: usize,
        impulse_id: usize,
        parent: BodyPartHandle,
        parent_internal_id: usize,
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
            multibody_handle: None,
            part_id,
            internal_id,
            assembly_id,
            impulse_id,
            is_leaf,
            parent,
            parent_internal_id,
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

    /// Checks if this link is the root of the multibody.
    #[inline]
    pub fn is_root(&self) -> bool {
        self.internal_id == 0
    }

    /// Reference to the joint attaching this link to its parent.
    #[inline]
    pub fn joint(&self) -> &Joint<N> {
        &*self.dof
    }

    /// Mutable reference to the joint attaching this link to its parent.
    #[inline]
    pub fn joint_mut(&mut self) -> &mut Joint<N> {
        &mut *self.dof
    }
}


impl<N: Real> BodyPart<N> for MultibodyLink<N> {
    #[inline]
    fn is_ground(&self) -> bool {
        false
    }

    #[inline]
    fn handle(&self) -> Option<BodyPartHandle> {
        self.multibody_handle.map(|h| BodyPartHandle::new(h, self.part_id))
    }

    #[inline]
    fn center_of_mass(&self) -> Point<N> {
        self.com
    }

    #[inline]
    fn velocity(&self) -> Velocity<N> {
        self.velocity
    }

    #[inline]
    fn position(&self) -> Isometry<N> {
        self.local_to_world
    }

    #[inline]
    fn local_inertia(&self) -> Inertia<N> {
        self.local_inertia
    }

    #[inline]
    fn inertia(&self) -> Inertia<N> {
        self.inertia
    }

    #[inline]
    fn apply_force(&mut self, force: &Force<N>) {
        self.external_forces += *force;
    }
}


// FIXME: keep this even if we already have the Index2 traits?
pub(crate) struct MultibodyLinkVec<N: Real>(pub Vec<MultibodyLink<N>>);

impl<N: Real> MultibodyLinkVec<N> {
    // #[inline]
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
        let parent_id = self[i].parent_internal_id;

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
