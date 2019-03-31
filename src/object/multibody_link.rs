use std::ops::{Deref, DerefMut};

use na::RealField;

use crate::joint::Joint;
use crate::math::{Inertia, Isometry, Point, Vector, Velocity};
use crate::object::{BodyPartHandle, BodyPart, BodyHandle};

/// One link of a multibody.
pub struct MultibodyLink<N: RealField> {
    pub(crate) name: String,
    // FIXME: make all those private.
    pub(crate) multibody_handle: BodyHandle,
    pub(crate) internal_id: usize,
    pub(crate) assembly_id: usize,
    pub(crate) impulse_id: usize,
    pub(crate) is_leaf: bool,

    // XXX: rename to "joint".
    // (And rename the full-coordinates joint constraints JointConstraint).
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
    pub(crate) inertia: Inertia<N>,
    pub(crate) com: Point<N>,

    pub(crate) local_inertia: Inertia<N>,
    pub(crate) local_com: Point<N>,
    // TODO: User-defined data
    // user_data:       T
}

impl<N: RealField> MultibodyLink<N> {
    /// Creates a new multibody link.
    pub fn new(
        internal_id: usize,
        assembly_id: usize,
        impulse_id: usize,
        multibody_handle: BodyHandle,
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
        let inertia = local_inertia.transformed(&local_to_world);
        let com = local_to_world * local_com;

        MultibodyLink {
            name: String::new(),
            multibody_handle,
            internal_id,
            assembly_id,
            impulse_id,
            is_leaf,
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
            local_inertia,
            local_com,
            inertia,
            com
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

    /// This link's name.
    #[inline]
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Sets this link's name.
    #[inline]
    pub fn set_name(&mut self, name: String) {
        self.name = name
    }

    /// The handle of this multibody link.
    #[inline]
    pub fn part_handle(&self) -> BodyPartHandle {
        BodyPartHandle(self.multibody_handle, self.internal_id)
    }
}


impl<N: RealField> BodyPart<N> for MultibodyLink<N> {
    #[inline]
    fn is_ground(&self) -> bool {
        false
    }

    #[inline]
    fn part_handle(&self) -> BodyPartHandle {
        BodyPartHandle(self.multibody_handle, self.internal_id)
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
}


// FIXME: keep this even if we already have the Index2 traits?
pub(crate) struct MultibodyLinkVec<N: RealField>(pub Vec<MultibodyLink<N>>);

impl<N: RealField> MultibodyLinkVec<N> {
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
}

impl<N: RealField> Deref for MultibodyLinkVec<N> {
    type Target = Vec<MultibodyLink<N>>;

    #[inline]
    fn deref(&self) -> &Vec<MultibodyLink<N>> {
        let MultibodyLinkVec(ref me) = *self;
        me
    }
}

impl<N: RealField> DerefMut for MultibodyLinkVec<N> {
    #[inline]
    fn deref_mut(&mut self) -> &mut Vec<MultibodyLink<N>> {
        let MultibodyLinkVec(ref mut me) = *self;
        me
    }
}
