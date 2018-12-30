use std::ops::Range;
use slab::Slab;
use std::iter;

use ncollide::shape::DeformationsType;
use ncollide::utils::IsometryOps;
use crate::joint::{Joint, FreeJoint};
use crate::math::{
    AngularDim, Dim, Force, Inertia, Isometry, Jacobian, Point, SpatialMatrix, SpatialVector,
    Vector, Velocity, DIM, Translation
};
use na::{self, DMatrix, DVector, DVectorSlice, DVectorSliceMut, Dynamic, MatrixMN, Real, LU};
use crate::object::{
    ActivationStatus, BodyPartHandle, BodyStatus, MultibodyLink,
    MultibodyLinkVec, Body, BodyPart, BodyHandle, RigidBody, Ground,
};
use crate::solver::{
    ConstraintSet, IntegrationParameters, MultibodyJointLimitsNonlinearConstraintGenerator, ForceDirection,
};
use crate::utils::{GeneralizedCross, IndexMut2};

/// An articulated body simulated using the reduced-coordinates approach.
pub struct Multibody<N: Real> {
    handle: Option<BodyHandle>,
    ids: Slab<usize>,
    rbs: MultibodyLinkVec<N>,
    velocities: Vec<N>,
    damping: Vec<N>,
    accelerations: Vec<N>,
    impulses: Vec<N>,
    body_jacobians: Vec<Jacobian<N>>,
    // FIXME: use sparse matrices.
    augmented_mass: DMatrix<N>,
    inv_augmented_mass: LU<N, Dynamic, Dynamic>,
    status: BodyStatus,
    activation: ActivationStatus<N>,
    ndofs: usize,
    companion_id: usize,
    unilateral_ground_rng: Range<usize>,
    bilateral_ground_rng: Range<usize>,

    /*
     * Workspaces.
     */
    coriolis_v: Vec<MatrixMN<N, Dim, Dynamic>>,
    coriolis_w: Vec<MatrixMN<N, AngularDim, Dynamic>>,
    i_coriolis_dt: Jacobian<N>,
}

impl<N: Real> Multibody<N> {
    /// Creates a new multibody with no link.
    pub fn new() -> Self {
        Multibody {
            handle: None,
            ids: Slab::new(),
            rbs: MultibodyLinkVec(Vec::new()),
            velocities: Vec::new(),
            damping: Vec::new(),
            accelerations: Vec::new(),
            impulses: Vec::new(),
            body_jacobians: Vec::new(),
            augmented_mass: DMatrix::zeros(0, 0),
            inv_augmented_mass: LU::new(DMatrix::zeros(0, 0)),
            status: BodyStatus::Dynamic,
            activation: ActivationStatus::new_active(),
            ndofs: 0,
            companion_id: 0,
            unilateral_ground_rng: 0..0,
            bilateral_ground_rng: 0..0,
            coriolis_v: Vec::new(),
            coriolis_w: Vec::new(),
            i_coriolis_dt: Jacobian::zeros(0),
        }
    }

    pub(crate) fn rbs(&self) -> &MultibodyLinkVec<N> {
        &self.rbs
    }

    pub(crate) fn rbs_mut(&mut self) -> &mut MultibodyLinkVec<N> {
        &mut self.rbs
    }

    /// Reference to the multibody link with the given handle.
    ///
    /// Return `None` if the given handle does not identifies a multibody link part of `self`.
    #[inline]
    pub fn link(&self, handle: BodyPartHandle) -> Option<&MultibodyLink<N>> {
        if self.handle? == handle.body_handle {
            let internal_id = *self.ids.get(handle.part_id)?;
            Some(&self.rbs[internal_id])
        } else {
            None
        }
    }

    /// Mutable reference to the multibody link with the given handle.
    ///
    /// Return `None` if the given handle does not identifies a multibody link part of `self`.
    #[inline]
    pub fn link_mut(&mut self, handle: BodyPartHandle) -> Option<&mut MultibodyLink<N>> {
        if self.handle? == handle.body_handle {
            let internal_id = *self.ids.get(handle.part_id)?;
            Some(&mut self.rbs[internal_id])
        } else {
            None
        }
    }

    /// Iterator through all the links of this multibody.alloc_jemalloc
    ///
    /// All link are guaranteed to be yielded before its descendant.
    pub fn links(&self) -> MultibodyLinks<N> {
        MultibodyLinks {
            mb: self,
            curr: 0,
        }
    }

    /// The slice of generalized velocities of this multibody.
    #[inline]
    pub fn generalized_velocity_slice(&self) -> &[N] {
        &self.velocities
    }

    /// The mutable slice of generalized velocities of this multibody.
    #[inline]
    pub fn generalized_velocity_slice_mut(&mut self) -> &mut [N] {
        &mut self.velocities
    }

    /// Mutable informations regarding activation and deactivation (sleeping) of this multibody.
    #[inline]
    fn activation_status_mut(&mut self) -> &mut ActivationStatus<N> {
        &mut self.activation
    }

    /// The vector of damping applied to this multibody.
    #[inline]
    pub fn damping(&self) -> DVectorSlice<N> {
        DVectorSlice::from_slice(&self.damping, self.ndofs)
    }

    /// Mutable vector of damping applied to this multibody.
    #[inline]
    pub fn damping_mut(&mut self) -> DVectorSliceMut<N> {
        DVectorSliceMut::from_slice(&mut self.damping, self.ndofs)
    }

    /// Generalized impulses applied to each degree of freedom.
    #[inline]
    pub fn impulses(&self) -> DVectorSlice<N> {
        DVectorSlice::from_slice(&self.impulses, self.impulses.len())
    }

    pub(crate) fn add_link<J: Joint<N>>(
        &mut self,
        parent: BodyPartHandle,
        mut dof: J,
        parent_shift: Vector<N>,
        body_shift: Vector<N>,
        local_inertia: Inertia<N>,
        local_com: Point<N>,
    ) -> &mut MultibodyLink<N> {
        assert!(
            parent.is_ground() || !self.rbs.is_empty(),
            "Multibody::build_body: invalid parent id."
        );

        /*
         * Compute the indices.
         */
        let assembly_id = self.velocities.len();
        let impulse_id = self.impulses.len();
        let internal_id = self.rbs.len();
        let part_id = self.ids.insert(internal_id);

        /*
         * Grow the buffers.
         */
        let ndofs = dof.ndofs();
        let nimpulses = dof.nimpulses();
        self.grow_buffers(ndofs, nimpulses);
        self.ndofs += ndofs;

        /*
         * Setup default damping.
         */
        dof.default_damping(&mut self.damping_mut().rows_mut(assembly_id, ndofs));

        /*
         * Create the multibody.
         */
        dof.update_jacobians(&body_shift, &self.velocities[assembly_id..]);
        let local_to_parent = dof.body_to_parent(&parent_shift, &body_shift);
        let local_to_world;
        let parent_to_world;

        let parent_internal_id;
        if !parent.is_ground() {
            parent_internal_id = *self.ids.get(parent.part_id).expect("Invalid parent handle.");
            let parent_rb = &mut self.rbs[parent_internal_id];
            parent_rb.is_leaf = false;
            parent_to_world = parent_rb.local_to_world;
            local_to_world = parent_rb.local_to_world * local_to_parent;
        } else {
            parent_internal_id = 0;
            parent_to_world = Isometry::identity();
            local_to_world = local_to_parent;
        }

        let mut rb = MultibodyLink::new(
            part_id,
            internal_id,
            assembly_id,
            impulse_id,
            parent,
            parent_internal_id,
            Box::new(dof),
            parent_shift,
            body_shift,
            parent_to_world,
            local_to_world,
            local_to_parent,
            local_inertia,
            local_com,
        );

        rb.multibody_handle = self.handle;
        self.rbs.push(rb);

        &mut self.rbs[internal_id]
    }

    fn grow_buffers(&mut self, ndofs: usize, nimpulses: usize) {
        let len = self.velocities.len();
        self.velocities.resize(len + ndofs, N::zero());
        self.damping.resize(len + ndofs, N::zero());
        self.accelerations.resize(len + ndofs, N::zero());
        self.body_jacobians.push(Jacobian::zeros(0));

        let len = self.impulses.len();
        self.impulses.resize(len + nimpulses, N::zero());
    }

    fn take_link(
        &mut self,
        mut link: MultibodyLink<N>,
        vels: &[N],
        damping: &[N],
    ) -> usize {
        let ndofs = link.dof.ndofs();
        let nimpulses = link.dof.nimpulses();
        let assembly_id = self.velocities.len();
        let impulse_id = self.impulses.len();
        let internal_id = self.rbs.len();

        self.ndofs += ndofs;
        self.grow_buffers(ndofs, nimpulses);
        self.velocities[assembly_id..].copy_from_slice(vels);
        self.damping[assembly_id..].copy_from_slice(damping);

        link.assembly_id = assembly_id;
        link.impulse_id = impulse_id;
        link.is_leaf = true;

        if !link.parent.is_ground() {
            assert!(
                link.parent_internal_id < self.rbs.len(),
                "Internal error: invalid parent."
            );
            self.rbs[link.parent_internal_id].is_leaf = false;
        }

        self.rbs.push(link);

        internal_id
    }

    /// Remove a set of links from this multibody.
    pub fn remove_links(self, links: &[BodyPartHandle]) -> Vec<Multibody<N>> {
        let mut rb2mb: Vec<_> = iter::repeat(0).take(self.rbs.len()).collect();
        let mut rb2id: Vec<_> = iter::repeat(0)
            .take(self.rbs.len())
            .collect();
        let mut removed: Vec<_> = iter::repeat(false).take(self.rbs.len()).collect();
        let mut multibodies = Vec::new();

        for link in links {
            removed[link.part_id] = true;
        }

        for (i, mut rb) in self.rbs.unwrap().into_iter().enumerate() {
            if !removed[i] {
                if rb.parent.is_ground() {
                    let ndofs = rb.dof.ndofs();
                    let velocities = &self.velocities[..ndofs];
                    let damping = &self.damping[..ndofs];
                    let mut mb = Multibody::new();

                    mb.status = self.status;
                    rb2id[i] = mb.take_link(rb, velocities, damping);
                    rb2mb[i] = multibodies.len();
                    multibodies.push(mb);
                } else if removed[rb.parent.part_id] {
                    let velocity = rb.velocity;
                    let damping = SpatialVector::zeros();
                    let mut mb = Multibody::new();

                    rb.parent = BodyPartHandle::ground();
                    rb.dof = Box::new(FreeJoint::new(rb.local_to_world));
                    rb.parent_shift.fill(N::zero());
                    rb.body_shift.fill(N::zero());

                    rb2id[i] = mb.take_link(rb, velocity.as_slice(), damping.as_slice());
                    rb2mb[i] = multibodies.len();
                    multibodies.push(mb);
                } else {
                    let parent_id = rb.parent.part_id;
                    let mb_id = rb2mb[parent_id];
                    let ndofs = rb.dof.ndofs();
                    let velocities = &self.velocities[rb.assembly_id..rb.assembly_id + ndofs];
                    let damping = &self.damping[rb.assembly_id..rb.assembly_id + ndofs];
                    let mut mb = &mut multibodies[mb_id];

                    rb.parent = BodyPartHandle::new(BodyHandle::ground(), rb2id[parent_id]);
                    rb2id[i] = mb.take_link(rb, velocities, damping);
                    rb2mb[i] = mb_id;
                }
            }
        }

        multibodies
    }

    /// Computes the constant terms of the dynamics.
    pub fn update_dynamics2(
        &mut self,
        gravity: &Vector<N>,
        params: &IntegrationParameters<N>,
        workspace: &mut MultibodyWorkspace<N>,
    ) {
        /*
         * Compute velocities.
         */
        {
            let rb = &mut self.rbs[0];
            let velocity_wrt_joint = rb
                .dof
                .jacobian_mul_coordinates(&self.velocities[rb.assembly_id..]);
            let velocity_dot_wrt_joint = rb
                .dof
                .jacobian_dot_mul_coordinates(&self.velocities[rb.assembly_id..]);

            rb.velocity_dot_wrt_joint = velocity_dot_wrt_joint;
            rb.velocity_wrt_joint = velocity_wrt_joint;
            rb.velocity = rb.velocity_wrt_joint;
        }

        for i in 1..self.rbs.len() {
            let (rb, parent_rb) = self.rbs.get_mut_with_parent(i);

            let velocity_wrt_joint = rb
                .dof
                .jacobian_mul_coordinates(&self.velocities[rb.assembly_id..]);
            let velocity_dot_wrt_joint = rb
                .dof
                .jacobian_dot_mul_coordinates(&self.velocities[rb.assembly_id..]);

            rb.velocity_dot_wrt_joint =
                velocity_dot_wrt_joint.transformed(&parent_rb.local_to_world);
            rb.velocity_wrt_joint = velocity_wrt_joint.transformed(&parent_rb.local_to_world);
            rb.velocity = parent_rb.velocity + rb.velocity_wrt_joint;

            let shift = rb.center_of_mass() - parent_rb.center_of_mass();
            rb.velocity.linear += parent_rb.velocity.angular_vector().gcross(&shift);
        }

        if self.status != BodyStatus::Dynamic {
            return;
        }

        /*
         * Update augmented mass matrix.
         */
        self.update_inertias(params);

        /*
         * Compute external forces.
         */
        workspace.resize(self.rbs.len());

        let mut accs = DVectorSliceMut::from_slice(&mut self.accelerations, self.ndofs);
        accs.fill(N::zero());

        for i in 0..self.rbs.len() {
            let external_forces;
            {
                let rb = &self.rbs[i];

                let mut acc = rb.velocity_dot_wrt_joint;

                if i != 0 {
                    let parent_id = rb.parent_internal_id;
                    let parent_rb = &self.rbs[parent_id];
                    let parent_vel = &parent_rb.velocity;

                    acc += workspace.accs[parent_id];
                    acc.linear += parent_vel
                        .angular_vector()
                        .gcross(&rb.velocity_wrt_joint.linear);
                    #[cfg(feature = "dim3")]
                        {
                            acc.angular += parent_vel.angular.cross(&rb.velocity_wrt_joint.angular);
                        }

                    let shift = rb.center_of_mass() - parent_rb.center_of_mass();
                    let dvel = rb.velocity.linear - parent_rb.velocity.linear;

                    acc.linear += parent_rb.velocity.angular_vector().gcross(&dvel);
                    acc.linear += workspace.accs[parent_id].angular_vector().gcross(&shift);
                }

                workspace.accs[i] = acc;

                let gravity_force = gravity * rb.inertia.mass();
                let gyroscopic;

                #[cfg(feature = "dim3")]
                    {
                        gyroscopic = rb
                            .velocity
                            .angular
                            .cross(&(rb.inertia.angular * rb.velocity.angular));
                    }
                #[cfg(feature = "dim2")]
                    {
                        gyroscopic = N::zero();
                    }

                external_forces = Force::new(gravity_force, -gyroscopic) - rb.inertia * acc;
                accs.gemv_tr(
                    N::one(),
                    &self.body_jacobians[i],
                    external_forces.as_vector(),
                    N::one(),
                );
            }
            self.rbs[i].external_forces += external_forces;
        }

        let damping = DVectorSlice::from_slice(&self.damping, self.ndofs);
        let vels = DVectorSlice::from_slice(&self.velocities, self.ndofs);
        accs.cmpy(-N::one(), &damping, &vels, N::one());

        assert!(self.inv_augmented_mass.solve_mut(&mut accs));
        // if accs.nrows() > 20 {
        //     println!(
        //         "Mass matrix: {}",
        //         self.augmented_mass
        //             .slice_with_steps((0, 0), (20, 20), (2, 2))
        //             .into_owned()
        //     );
        //     println!(
        //         "Accelerations: {}",
        //         accs.rows_with_step(0, 20, 2).into_owned()
        //     );
        // } else {
        //     println!("Mass matrix: {}", self.augmented_mass);
        //     println!("Accelerations: {}", accs);
        // }
    }

    fn update_body_jacobians(&mut self) {
        for i in 0..self.rbs.len() {
            let rb = &self.rbs[i];

            if self.body_jacobians[i].ncols() != self.ndofs {
                // FIXME: use a resize instead.
                self.body_jacobians[i] = Jacobian::zeros(self.ndofs);
            }

            if i != 0 {
                let parent_id = rb.parent_internal_id;
                let parent_rb = &self.rbs[parent_id];
                let (rb_j, parent_j) = self.body_jacobians.index_mut_const(i, parent_id);
                rb_j.copy_from(&parent_j);

                {
                    let mut rb_j_v = rb_j.fixed_rows_mut::<Dim>(0);
                    let parent_j_w = parent_j.fixed_rows::<AngularDim>(DIM);

                    let shift_tr =
                        (rb.center_of_mass() - parent_rb.center_of_mass()).gcross_matrix_tr();
                    rb_j_v.gemm(N::one(), &shift_tr, &parent_j_w, N::one());
                }
            } else {
                self.body_jacobians[i].fill(N::zero());
            }

            let ndofs = rb.dof.ndofs();
            let mut tmp = SpatialMatrix::zeros();
            let mut rb_joint_j = tmp.columns_mut(0, ndofs);
            let mut rb_j_part = self.body_jacobians[i].columns_mut(rb.assembly_id, ndofs);
            rb.dof.jacobian(&rb.parent_to_world, &mut rb_joint_j);

            rb_j_part += rb_joint_j;
        }
    }

    fn update_inertias(&mut self, params: &IntegrationParameters<N>) {
        if self.augmented_mass.ncols() != self.ndofs {
            // FIXME: do a resize instead of a full reallocation.
            self.augmented_mass = DMatrix::zeros(self.ndofs, self.ndofs);
        }

        for i in 0..self.rbs.len() {
            let mut rb = &mut self.rbs[i];
            rb.inertia = rb.local_inertia.transformed(&rb.local_to_world);
        }

        if self.coriolis_v.len() != self.rbs.len() {
            self.coriolis_v.resize(
                self.rbs.len(),
                MatrixMN::<N, Dim, Dynamic>::zeros(self.ndofs),
            );
            self.coriolis_w.resize(
                self.rbs.len(),
                MatrixMN::<N, AngularDim, Dynamic>::zeros(self.ndofs),
            );
            self.i_coriolis_dt = Jacobian::zeros(self.ndofs);
        }

        for i in 0..self.rbs.len() {
            let rb = &self.rbs[i];
            let body_jacobian = &self.body_jacobians[i];

            let mut augmented_inertia = rb.inertia;

            #[cfg(feature = "dim3")]
                {
                    // Derivative of gyroscopic forces.
                    let ang_inertia = rb.inertia.angular;
                    let gyroscopic_matrix = rb.velocity.angular.gcross_matrix() * ang_inertia
                        - (ang_inertia * rb.velocity.angular).gcross_matrix();

                    augmented_inertia.angular += gyroscopic_matrix * params.dt;
                }

            // FIXME: optimize that (knowing the structure of the augmented inertia matrix).
            // FIXME: this could be better optimized in 2D.
            self.augmented_mass.quadform(
                N::one(),
                &augmented_inertia.to_matrix(),
                body_jacobian,
                N::one(),
            );

            /*
             *
             * Coriolis matrix.
             *
             */
            let rb_j = &self.body_jacobians[i];
            let rb_j_v = rb_j.fixed_rows::<Dim>(0);

            let ndofs = rb.dof.ndofs();

            if i != 0 {
                let parent_id = rb.parent_internal_id;
                let parent_rb = &self.rbs[parent_id];
                let parent_j = &self.body_jacobians[parent_id];
                let parent_j_v = parent_j.fixed_rows::<Dim>(0);
                let parent_j_w = parent_j.fixed_rows::<AngularDim>(DIM);
                let parent_w = parent_rb.velocity.angular_vector().gcross_matrix();

                let (coriolis_v, parent_coriolis_v) = self.coriolis_v.index_mut_const(i, parent_id);
                let (coriolis_w, parent_coriolis_w) = self.coriolis_w.index_mut_const(i, parent_id);

                // JDot + JDot/u * qdot
                coriolis_v.copy_from(&parent_coriolis_v);
                coriolis_w.copy_from(&parent_coriolis_w);

                let shift_tr =
                    (rb.center_of_mass() - parent_rb.center_of_mass()).gcross_matrix_tr();
                coriolis_v.gemm(N::one(), &shift_tr, &parent_coriolis_w, N::one());

                // JDot
                let dvel_tr = (rb.velocity.linear - parent_rb.velocity.linear).gcross_matrix_tr();
                coriolis_v.gemm(N::one(), &dvel_tr, &parent_j_w, N::one());

                // JDot/u * qdot
                coriolis_v.gemm(
                    N::one(),
                    &rb.velocity_wrt_joint.linear.gcross_matrix_tr(),
                    &parent_j_w,
                    N::one(),
                );
                coriolis_v.gemm(N::one(), &parent_w, &rb_j_v, N::one());
                coriolis_v.gemm(-N::one(), &parent_w, &parent_j_v, N::one());

                #[cfg(feature = "dim3")]
                    {
                        let vel_wrt_joint_w = rb.velocity_wrt_joint.angular_vector().gcross_matrix();
                        coriolis_w.gemm(-N::one(), &vel_wrt_joint_w, &parent_j_w, N::one());
                    }

                {
                    let mut coriolis_v_part = coriolis_v.columns_mut(rb.assembly_id, ndofs);

                    let mut tmp1 = SpatialMatrix::zeros();
                    let mut rb_joint_j = tmp1.columns_mut(0, ndofs);
                    rb.dof.jacobian(&parent_rb.local_to_world, &mut rb_joint_j);

                    let rb_joint_j_v = rb_joint_j.fixed_rows::<Dim>(0);

                    // JDot
                    coriolis_v_part.gemm(N::one(), &parent_w, &rb_joint_j_v, N::one());

                    #[cfg(feature = "dim3")]
                        {
                            let rb_joint_j_w = rb_joint_j.fixed_rows::<AngularDim>(DIM);
                            let mut coriolis_w_part = coriolis_w.columns_mut(rb.assembly_id, ndofs);
                            coriolis_w_part.gemm(N::one(), &parent_w, &rb_joint_j_w, N::one());
                        }
                }
            } else {
                self.coriolis_v[i].fill(N::zero());
                self.coriolis_w[i].fill(N::zero());
            }

            let coriolis_v = &mut self.coriolis_v[i];
            let coriolis_w = &mut self.coriolis_w[i];

            {
                let mut tmp1 = SpatialMatrix::zeros();
                let mut tmp2 = SpatialMatrix::zeros();
                let mut rb_joint_j_dot = tmp1.columns_mut(0, ndofs);
                let mut rb_joint_j_dot_veldiff = tmp2.columns_mut(0, ndofs);

                rb.dof
                    .jacobian_dot(&rb.parent_to_world, &mut rb_joint_j_dot);
                rb.dof.jacobian_dot_veldiff_mul_coordinates(
                    &rb.parent_to_world,
                    &self.velocities[rb.assembly_id..],
                    &mut rb_joint_j_dot_veldiff,
                );

                let rb_joint_j_v_dot = rb_joint_j_dot.fixed_rows::<Dim>(0);
                let rb_joint_j_w_dot = rb_joint_j_dot.fixed_rows::<AngularDim>(DIM);
                let rb_joint_j_v_dot_veldiff = rb_joint_j_dot_veldiff.fixed_rows::<Dim>(0);
                let rb_joint_j_w_dot_veldiff = rb_joint_j_dot_veldiff.fixed_rows::<AngularDim>(DIM);

                let mut coriolis_v_part = coriolis_v.columns_mut(rb.assembly_id, ndofs);
                let mut coriolis_w_part = coriolis_w.columns_mut(rb.assembly_id, ndofs);

                // JDot
                coriolis_v_part += rb_joint_j_v_dot;
                coriolis_w_part += rb_joint_j_w_dot;

                // JDot/u * qdot
                coriolis_v_part += rb_joint_j_v_dot_veldiff;
                coriolis_w_part += rb_joint_j_w_dot_veldiff;
            }

            /*
             * Meld with the mass matrix.
             */
            {
                let mut i_coriolis_dt_v = self.i_coriolis_dt.fixed_rows_mut::<Dim>(0);
                i_coriolis_dt_v.copy_from(coriolis_v);
                i_coriolis_dt_v *= rb.inertia.linear * params.dt;
            }

            {
                // FIXME: in 2D this is just an axpy.
                let mut i_coriolis_dt_w = self.i_coriolis_dt.fixed_rows_mut::<AngularDim>(DIM);
                i_coriolis_dt_w.gemm(
                    params.dt,
                    rb.inertia.angular_matrix(),
                    &coriolis_w,
                    N::zero(),
                );
            }

            self.augmented_mass
                .gemm_tr(N::one(), &rb_j, &self.i_coriolis_dt, N::one());
        }

        /*
         * Damping.
         */
        for i in 0..self.ndofs {
            self.augmented_mass[(i, i)] += self.damping[i] * params.dt;
        }

        // FIXME: avoid allocation inside LU at each timestep.
        self.inv_augmented_mass = LU::new(self.augmented_mass.clone());
    }

    /// The generalized velocity at the joint of the given link.
    #[inline]
    pub fn joint_velocity(&self, link: &MultibodyLink<N>) -> DVectorSlice<N> {
        let ndofs = link.dof.ndofs();
        DVectorSlice::from_slice(
            &self.velocities[link.assembly_id..link.assembly_id + ndofs],
            ndofs,
        )
    }

    /// Convert a force applied to the center of mass of the link `rb_id` into generalized force.
    pub fn link_jacobian_mul_force(&self, link: &MultibodyLink<N>, force: &Force<N>, out: &mut [N]) {
        let mut out = DVectorSliceMut::from_slice(out, self.ndofs);
        self.body_jacobians[link.internal_id].tr_mul_to(force.as_vector(), &mut out);
    }

    /// Convert a force applied to this multibody's link `rb_id` center of mass into generalized accelerations.
    pub fn inv_mass_mul_link_force(&self, link: &MultibodyLink<N>, force: &Force<N>, out: &mut [N]) {
        let mut out = DVectorSliceMut::from_slice(out, self.ndofs);
        self.body_jacobians[link.internal_id].tr_mul_to(force.as_vector(), &mut out);
        assert!(self.inv_augmented_mass.solve_mut(&mut out));
    }

    /// Convert a generalized force applied to le link `rb_id`'s degrees of freedom into generalized accelerations.
    ///
    /// The joint attaching this link to its parent is assumed to be a unit joint.
    pub fn inv_mass_mul_unit_joint_force(
        &self,
        link: &MultibodyLink<N>,
        dof_id: usize,
        force: N,
        out: &mut [N],
    ) {
        let mut out = DVectorSliceMut::from_slice(out, self.ndofs);
        out.fill(N::zero());
        out[link.assembly_id + dof_id] = force;
        assert!(self.inv_augmented_mass.solve_mut(&mut out));
    }

    /// Convert a generalized force applied to the link `rb_id`'s degrees of freedom into generalized accelerations.
    pub fn inv_mass_mul_joint_force(
        &self,
        link: &MultibodyLink<N>,
        force: DVectorSlice<N>,
        out: &mut [N],
    ) {
        let ndofs = link.dof.ndofs();

        let mut out = DVectorSliceMut::from_slice(out, self.ndofs);
        out.fill(N::zero());
        out.rows_mut(link.assembly_id, ndofs).copy_from(&force);
        assert!(self.inv_augmented_mass.solve_mut(&mut out));
    }

    /// The augmented mass (inluding gyroscropic and coriolis terms) in world-space of this multibody.
    pub fn augmented_mass(&self) -> &DMatrix<N> {
        &self.augmented_mass
    }

    /// Retrieve the mutable generalized velocities of this link.
    #[inline]
    pub fn joint_velocity_mut(&mut self, handle: BodyPartHandle) -> DVectorSliceMut<N> {
        let ndofs;
        let i;
        {
            let link = self.link(handle).expect("Invalid multibody link handle.");
            ndofs = link.dof.ndofs();
            i = link.assembly_id;
        }

        DVectorSliceMut::from_slice(&mut self.velocities[i..i + ndofs], ndofs)
    }

    /// Generates the set of velocity and position constraints needed for joint limits and motors at each link
    /// of this multibody.
    pub fn constraints(
        &mut self,
        params: &IntegrationParameters<N>,
        ext_vels: &DVector<N>,
        ground_j_id: &mut usize,
        jacobians: &mut [N],
        constraints: &mut ConstraintSet<N>,
    ) {
        let first_unilateral = constraints.velocity.unilateral_ground.len();
        let first_bilateral = constraints.velocity.bilateral_ground.len();

        for link in self.links() {
            link.joint().velocity_constraints(
                params,
                self,
                &link,
                self.companion_id,
                0,
                ext_vels.as_slice(),
                ground_j_id,
                jacobians,
                constraints,
            );

            if link.joint().num_position_constraints() != 0 {
                let generator =
                    MultibodyJointLimitsNonlinearConstraintGenerator::new(link.handle().unwrap());
                constraints.position.multibody_limits.push(generator)
            }
        }

        self.unilateral_ground_rng = first_unilateral..constraints.velocity.unilateral_ground.len();
        self.bilateral_ground_rng = first_bilateral..constraints.velocity.bilateral_ground.len();
    }

    /// Store impulses computed by the solver for joint limits and motors.
    pub fn cache_impulses(&mut self, constraints: &ConstraintSet<N>) {
        for constraint in
            &constraints.velocity.unilateral_ground[self.unilateral_ground_rng.clone()]
            {
                self.impulses[constraint.impulse_id] = constraint.impulse;
            }

        for constraint in &constraints.velocity.bilateral_ground[self.bilateral_ground_rng.clone()]
            {
                self.impulses[constraint.impulse_id] = constraint.impulse;
            }
    }
}

/// Iterator through all the multibody links.
pub struct MultibodyLinks<'a, N: Real> {
    mb: &'a Multibody<N>,
    curr: usize,
}

impl<'a, N: Real> Iterator for MultibodyLinks<'a, N> {
    type Item = &'a MultibodyLink<N>;

    #[inline]
    fn next(&mut self) -> Option<&'a MultibodyLink<N>> {
        if self.curr < self.mb.rbs.len() {
            let res = &self.mb.rbs[self.curr];
            self.curr += 1;
            Some(res)
        } else {
            None
        }
    }
}

/// A temporary workspace for various updates of the multibody.
pub struct MultibodyWorkspace<N: Real> {
    accs: Vec<Velocity<N>>,
}

impl<N: Real> MultibodyWorkspace<N> {
    /// Create an empty workspace.
    pub fn new() -> Self {
        MultibodyWorkspace { accs: Vec::new() }
    }

    /// Resize the workspace so it is enough for `nlinks` links.
    pub fn resize(&mut self, nlinks: usize) {
        self.accs.resize(nlinks, Velocity::zero());
    }
}

impl<N: Real> Body<N> for Multibody<N> {
    #[inline]
    fn set_handle(&mut self, handle: Option<BodyHandle>) {
        self.handle = handle;

        for rb in &mut *self.rbs {
            rb.multibody_handle = handle;
        }

        if let Some(handle) = handle {
            if self.rbs.len() > 1 {
                for rb in &mut self.rbs[1..] {
                    rb.parent.body_handle = handle;
                }
            }
        }
    }

    #[inline]
    fn part(&self, handle: BodyPartHandle) -> &BodyPart<N> {
        self.link(handle).expect("Multibody part not found.")
    }

    #[inline]
    fn part_mut(&mut self, handle: BodyPartHandle) -> &mut BodyPart<N> {
        self.link_mut(handle).expect("Multibody part not found.")
    }

    #[inline]
    fn contains_part(&self, handle: BodyPartHandle) -> bool {
        self.link(handle).is_some()
    }

    #[inline]
    fn deformed_positions(&self) -> Option<(DeformationsType, &[N])> {
        None
    }

    #[inline]
    fn deformed_positions_mut(&mut self) -> Option<(DeformationsType, &mut [N])> {
        None
    }

    #[inline]
    fn integrate(&mut self, params: &IntegrationParameters<N>) {
        for rb in self.rbs.iter_mut() {
            rb.dof.integrate(params, &self.velocities[rb.assembly_id..])
        }
    }

    fn apply_displacement(&mut self, disp: &[N]) {
        for rb in self.rbs.iter_mut() {
            rb.dof.apply_displacement(&disp[rb.assembly_id..])
        }

        self.update_kinematics();
    }

    fn clear_dynamics(&mut self) {
        self.augmented_mass.fill(N::zero());
        let mut accs = DVectorSliceMut::from_slice(&mut self.accelerations, self.ndofs);
        accs.fill(N::zero());

        for rb in &mut *self.rbs {
            rb.external_forces = Force::zero();
        }
    }

    fn update_kinematics(&mut self) {
        // Special case for the root, which has no parent.
        {
            let rb = &mut self.rbs[0];
            rb.dof.update_jacobians(&rb.body_shift, &self.velocities);
            rb.local_to_parent = rb.dof.body_to_parent(&rb.parent_shift, &rb.body_shift);
            rb.local_to_world = rb.local_to_parent;
            rb.com = rb.local_to_world * rb.local_com;
        }

        // Handle the children. They all have a parent within this multibody.
        for i in 1..self.rbs.len() {
            let (rb, parent_rb) = self.rbs.get_mut_with_parent(i);

            rb.dof
                .update_jacobians(&rb.body_shift, &self.velocities[rb.assembly_id..]);
            rb.local_to_parent = rb.dof.body_to_parent(&rb.parent_shift, &rb.body_shift);
            rb.local_to_world = parent_rb.local_to_world * rb.local_to_parent;
            rb.parent_to_world = parent_rb.local_to_world;
            rb.com = rb.local_to_world * rb.local_com;
        }

        /*
         * Compute body jacobians.
         */
        self.update_body_jacobians();
    }

    fn update_dynamics(
        &mut self,
        gravity: &Vector<N>,
        params: &IntegrationParameters<N>,
    ) {
        // FIXME
    }

    #[inline]
    fn generalized_acceleration(&self) -> DVectorSlice<N> {
        DVectorSlice::from_slice(&self.accelerations, self.ndofs)
    }

    #[inline]
    fn generalized_velocity(&self) -> DVectorSlice<N> {
        DVectorSlice::from_slice(&self.velocities, self.ndofs)
    }

    #[inline]
    fn generalized_velocity_mut(&mut self) -> DVectorSliceMut<N> {
        DVectorSliceMut::from_slice(&mut self.velocities, self.ndofs)
    }

    #[inline]
    fn handle(&self) -> Option<BodyHandle> {
        self.handle
    }

    #[inline]
    fn activation_status(&self) -> &ActivationStatus<N> {
        &self.activation
    }

    #[inline]
    fn activate_with_energy(&mut self, energy: N) {
        self.activation.set_energy(energy)
    }

    #[inline]
    fn deactivate(&mut self) {
        self.activation.set_energy(N::zero());
        for v in &mut self.velocities {
            *v = N::zero()
        }
    }

    #[inline]
    fn set_deactivation_threshold(&mut self, threshold: Option<N>) {
        self.activation.set_deactivation_threshold(threshold)
    }

    #[inline]
    fn set_status(&mut self, status: BodyStatus) {
        self.status = status
    }


    #[inline]
    fn status(&self) -> BodyStatus {
        self.status
    }

    #[inline]
    fn companion_id(&self) -> usize {
        self.companion_id
    }

    #[inline]
    fn set_companion_id(&mut self, id: usize) {
        self.companion_id = id
    }

    #[inline]
    fn ndofs(&self) -> usize {
        self.ndofs
    }

    #[inline]
    fn world_point_at_material_point(&self, part: &BodyPart<N>, point: &Point<N>) -> Point<N> {
        let link = part.downcast_ref::<MultibodyLink<N>>().expect("The provided body part must be a multibody link");
        link.local_to_world * point
    }

    #[inline]
    fn position_at_material_point(&self, part: &BodyPart<N>, point: &Point<N>) -> Isometry<N> {
        let link = part.downcast_ref::<MultibodyLink<N>>().expect("The provided body part must be a multibody link");
        link.local_to_world * Translation::from_vector(point.coords)
    }

    #[inline]
    fn material_point_at_world_point(&self, part: &BodyPart<N>, point: &Point<N>) -> Point<N> {
        let link = part.downcast_ref::<MultibodyLink<N>>().expect("The provided body part must be a multibody link");
        link.local_to_world.inverse_transform_point(point)
    }

    fn fill_constraint_geometry(
        &self,
        part: &BodyPart<N>,
        ndofs: usize, // FIXME: keep this parameter?
        point: &Point<N>,
        force_dir: &ForceDirection<N>,
        j_id: usize,
        wj_id: usize,
        jacobians: &mut [N],
        inv_r: &mut N,
        ext_vels: Option<&DVectorSlice<N>>,
        out_vel: Option<&mut N>
    ) {
        let link = part.downcast_ref::<MultibodyLink<N>>().expect("The provided body part must be a multibody link");

        match self.status() {
            BodyStatus::Dynamic => {
                let pos = point - link.com.coords;
                let force = force_dir.at_point(&pos);

                self.link_jacobian_mul_force(link, &force, &mut jacobians[j_id..]);

                // FIXME: this could be optimized with a copy_nonoverlapping.
                for i in 0..ndofs {
                    jacobians[wj_id + i] = jacobians[j_id + i];
                }

                {
                    let mut out = DVectorSliceMut::from_slice(&mut jacobians[wj_id..], self.ndofs);
                    assert!(self.inv_augmented_mass.solve_mut(&mut out))
                }

                let j = DVectorSlice::from_slice(&jacobians[j_id..], ndofs);
                let invm_j = DVectorSlice::from_slice(&jacobians[wj_id..], ndofs);

                *inv_r += j.dot(&invm_j);

                if let Some(out_vel) = out_vel {
                    *out_vel += j.dot(&self.generalized_velocity());

                    if let Some(ext_vels) = ext_vels {
                        *out_vel += j.dot(ext_vels)
                    }
                }
            },
            BodyStatus::Kinematic => {
                if let Some(out_vel) = out_vel {
                    match *force_dir {
                        ForceDirection::Linear(ref normal) => {
                            let dpos = point - link.com;
                            *out_vel = link.velocity.shift(&dpos).linear.dot(normal)
                        }
                        ForceDirection::Angular(ref axis) => {
                            *out_vel = link.velocity.angular_vector().dot(axis)
                        }
                    }
                }
            },
            BodyStatus::Static | BodyStatus::Disabled => {}
        }
    }

    #[inline]
    fn has_active_internal_constraints(&mut self) -> bool {
        false
    }

    #[inline]
    fn setup_internal_velocity_constraints(&mut self, dvels: &mut DVectorSliceMut<N>) {}

    #[inline]
    fn step_solve_internal_velocity_constraints(&mut self, dvels: &mut DVectorSliceMut<N>) {
        // FIXME: solve joint limit/motor constraints here directly?
        // (instead of having a special case by returning the set of constraints to the solver).
    }

    #[inline]
    fn step_solve_internal_position_constraints(&mut self, params: &IntegrationParameters<N>) {}
}