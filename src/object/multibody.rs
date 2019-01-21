use std::ops::Range;
use std::any::Any;

use ncollide::shape::DeformationsType;
use ncollide::utils::IsometryOps;
use crate::joint::Joint;
use crate::math::{
    AngularDim, Dim, Force, Inertia, Isometry, Jacobian, Point, SpatialMatrix,
    Vector, Velocity, DIM, Translation
};
use na::{self, DMatrix, DVector, DVectorSlice, DVectorSliceMut, Dynamic, MatrixMN, Real, LU};
use crate::object::{
    ActivationStatus, BodyPartHandle, BodyStatus, MultibodyLink, BodyUpdateStatus,
    MultibodyLinkVec, Body, BodyPart, BodyHandle, ColliderDesc, BodyDesc
};
use crate::solver::{
    ConstraintSet, IntegrationParameters, MultibodyJointLimitsNonlinearConstraintGenerator, ForceDirection,
};
use crate::world::{World, ColliderWorld};
use crate::utils::{GeneralizedCross, IndexMut2};

/// An articulated body simulated using the reduced-coordinates approach.
pub struct Multibody<N: Real> {
    handle: BodyHandle,
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
    update_status: BodyUpdateStatus,
    activation: ActivationStatus<N>,
    ndofs: usize,
    companion_id: usize,
    unilateral_ground_rng: Range<usize>,
    bilateral_ground_rng: Range<usize>,
    user_data: Option<Box<Any + Send + Sync>>,

    /*
     * Workspaces.
     */
    workspace: MultibodyWorkspace<N>,
    coriolis_v: Vec<MatrixMN<N, Dim, Dynamic>>,
    coriolis_w: Vec<MatrixMN<N, AngularDim, Dynamic>>,
    i_coriolis_dt: Jacobian<N>,
}

impl<N: Real> Multibody<N> {
    /// Creates a new multibody with no link.
    fn new(handle: BodyHandle) -> Self {
        Multibody {
            handle,
            rbs: MultibodyLinkVec(Vec::new()),
            velocities: Vec::new(),
            damping: Vec::new(),
            accelerations: Vec::new(),
            impulses: Vec::new(),
            body_jacobians: Vec::new(),
            augmented_mass: DMatrix::zeros(0, 0),
            inv_augmented_mass: LU::new(DMatrix::zeros(0, 0)),
            status: BodyStatus::Dynamic,
            update_status: BodyUpdateStatus::new(),
            activation: ActivationStatus::new_active(),
            ndofs: 0,
            companion_id: 0,
            unilateral_ground_rng: 0..0,
            bilateral_ground_rng: 0..0,
            workspace: MultibodyWorkspace::new(),
            coriolis_v: Vec::new(),
            coriolis_w: Vec::new(),
            i_coriolis_dt: Jacobian::zeros(0),
            user_data: None
        }
    }

    user_data_accessors!();

    #[inline]
    pub fn root(&self) -> &MultibodyLink<N> {
        &self.rbs[0]
    }

    #[inline]
    pub fn root_mut(&mut self) -> &mut MultibodyLink<N> {
        &mut self.rbs[0]
    }

    /// Reference to the multibody link with the given handle.
    ///
    /// Return `None` if the given handle does not identifies a multibody link part of `self`.
    #[inline]
    pub fn link(&self, id: usize) -> Option<&MultibodyLink<N>> {
        self.rbs.get(id)
    }

    /// Mutable reference to the multibody link with the given id.
    ///
    /// Return `None` if the given id does not identifies a multibody link part of `self`.
    #[inline]
    pub fn link_mut(&mut self, id: usize) -> Option<&mut MultibodyLink<N>> {
        self.rbs.get_mut(id)
    }

    /// Iterator through all the links of this multibody.
    ///
    /// All link are guaranteed to be yielded before its descendant.
    pub fn links(&self) -> impl Iterator<Item = &MultibodyLink<N>> {
        self.rbs.iter()
    }

    /// The slice of generalized velocities of this multibody.
    #[inline]
    pub fn generalized_velocity_slice(&self) -> &[N] {
        &self.velocities
    }

    /// The mutable slice of generalized velocities of this multibody.
    #[inline]
    pub fn generalized_velocity_slice_mut(&mut self) -> &mut [N] {
        self.update_status.set_velocity_changed(true);
        &mut self.velocities
    }

    /// The vector of damping applied to this multibody.
    #[inline]
    pub fn damping(&self) -> DVectorSlice<N> {
        DVectorSlice::from_slice(&self.damping, self.ndofs)
    }

    /// Mutable vector of damping applied to this multibody.
    #[inline]
    pub fn damping_mut(&mut self) -> DVectorSliceMut<N> {
        self.update_status.set_damping_changed(true);
        DVectorSliceMut::from_slice(&mut self.damping, self.ndofs)
    }

    /// Generalized impulses applied to each degree of freedom.
    #[inline]
    pub fn impulses(&self) -> DVectorSlice<N> {
        DVectorSlice::from_slice(&self.impulses, self.impulses.len())
    }

    fn add_link(
        &mut self,
        parent: BodyPartHandle,
        mut dof: Box<Joint<N>>,
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
            parent_internal_id = parent.1;
            let parent_rb = &mut self.rbs[parent_internal_id];
            parent_rb.is_leaf = false;
            parent_to_world = parent_rb.local_to_world;
            local_to_world = parent_rb.local_to_world * local_to_parent;
        } else {
            parent_internal_id = 0;
            parent_to_world = Isometry::identity();
            local_to_world = local_to_parent;
        }

        let rb = MultibodyLink::new(
            internal_id,
            assembly_id,
            impulse_id,
            self.handle,
            parent_internal_id,
            dof,
            parent_shift,
            body_shift,
            parent_to_world,
            local_to_world,
            local_to_parent,
            local_inertia,
            local_com,
        );

        self.rbs.push(rb);
        self.workspace.resize(self.rbs.len());

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

    fn update_acceleration(&mut self, gravity: &Vector<N>) {
        if self.status != BodyStatus::Dynamic {
            return;
        }

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

                    acc += self.workspace.accs[parent_id];
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
                    acc.linear += self.workspace.accs[parent_id].angular_vector().gcross(&shift);
                }

                self.workspace.accs[i] = acc;

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

                external_forces = Force::new(gravity_force, -gyroscopic) - rb.inertia * acc + rb.external_forces;
                accs.gemv_tr(
                    N::one(),
                    &self.body_jacobians[i],
                    external_forces.as_vector(),
                    N::one(),
                );
            }
        }

        let damping = DVectorSlice::from_slice(&self.damping, self.ndofs);
        let vels = DVectorSlice::from_slice(&self.velocities, self.ndofs);
        accs.cmpy(-N::one(), &damping, &vels, N::one());

        assert!(self.inv_augmented_mass.solve_mut(&mut accs));
    }

    /// Computes the constant terms of the dynamics.
    fn update_dynamics(
        &mut self,
        gravity: &Vector<N>,
        params: &IntegrationParameters<N>
    ) {
        if !self.update_status.inertia_needs_update() {
            return;
        }

        /*
         * Compute velocities.
         */
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
        } else {
            self.augmented_mass.fill(N::zero());
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

            #[allow(unused_mut)] // mut is needed for 3D but not for 2D.
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
    pub fn joint_velocity_mut(&mut self, id: usize) -> DVectorSliceMut<N> {
        self.update_status.set_velocity_changed(true);
        let ndofs;
        let i;
        {
            let link = self.link(id).expect("Invalid multibody link handle.");
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
                    MultibodyJointLimitsNonlinearConstraintGenerator::new();
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

/// A temporary workspace for various updates of the multibody.
struct MultibodyWorkspace<N: Real> {
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
    fn part(&self, id: usize) -> Option<&BodyPart<N>> {
        self.link(id).map(|l| l as &BodyPart<N>)
    }

    #[inline]
    fn part_mut(&mut self, id: usize) -> Option<&mut BodyPart<N>> {
        self.link_mut(id).map(|l| l as &mut BodyPart<N>)
    }

    #[inline]
    fn deformed_positions(&self) -> Option<(DeformationsType, &[N])> {
        None
    }

    #[inline]
    fn deformed_positions_mut(&mut self) -> Option<(DeformationsType, &mut [N])> {
        None
    }

    fn clear_update_flags(&mut self) {
        self.update_status.clear();
    }

    fn update_status(&self) -> BodyUpdateStatus {
        self.update_status
    }

    #[inline]
    fn integrate(&mut self, params: &IntegrationParameters<N>) {
        self.update_status.set_position_changed(true);

        for rb in self.rbs.iter_mut() {
            rb.dof.integrate(params, &self.velocities[rb.assembly_id..])
        }
    }

    fn apply_displacement(&mut self, disp: &[N]) {
        self.update_status.set_position_changed(true);

        for rb in self.rbs.iter_mut() {
            rb.dof.apply_displacement(&disp[rb.assembly_id..])
        }

        self.update_kinematics();
    }

    fn clear_dynamics(&mut self) {
    }

    fn clear_forces(&mut self) {
        for rb in &mut *self.rbs {
            rb.external_forces = Force::zero();
        }
    }

    fn update_kinematics(&mut self) {
        if !self.update_status.position_changed() {
            return;
        }

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
        self.update_dynamics(gravity, params)
    }

    fn update_acceleration(&mut self, gravity: &Vector<N>, _: &IntegrationParameters<N>) {
        self.update_acceleration(gravity)
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
        self.update_status.set_velocity_changed(true);
        DVectorSliceMut::from_slice(&mut self.velocities, self.ndofs)
    }

    #[inline]
    fn handle(&self) -> BodyHandle {
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
        self.update_status.set_velocity_changed(true);
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
    fn setup_internal_velocity_constraints(&mut self, _: &mut DVectorSliceMut<N>) {}

    #[inline]
    fn step_solve_internal_velocity_constraints(&mut self, _: &mut DVectorSliceMut<N>) {
        // FIXME: solve joint limit/motor constraints here directly?
        // (instead of having a special case by returning the set of constraints to the solver).
    }

    #[inline]
    fn step_solve_internal_position_constraints(&mut self, _params: &IntegrationParameters<N>) {}
}


pub struct MultibodyDesc<'a, N: Real> {
    children: Vec<MultibodyDesc<'a, N>>,
    joint: Box<Joint<N>>,
    velocity: Velocity<N>,
    local_inertia: Inertia<N>,
    local_com: Point<N>,
    colliders: Vec<&'a ColliderDesc<N>>,
    body_shift: Vector<N>,
    parent_shift: Vector<N>
}

impl<'a, N: Real> MultibodyDesc<'a, N> {
    pub fn new<J: Joint<N>>(joint: J) -> Self {
        MultibodyDesc {
            joint: Box::new(joint),
            children: Vec::new(),
            velocity: Velocity::zero(),
            local_inertia: Inertia::zero(),
            local_com: Point::origin(),
            colliders: Vec::new(),
            body_shift: Vector::zeros(),
            parent_shift: Vector::zeros()
        }
    }

    pub fn add_child<J: Joint<N>>(&mut self, joint: J) -> &mut MultibodyDesc<'a, N> {
        let child = MultibodyDesc::new(joint);

        self.children.push(child);
        self.children.last_mut().unwrap()
    }

    pub fn set_joint<J: Joint<N>>(&mut self, joint: J) -> &mut Self {
        self.joint = Box::new(joint);
        self
    }

    desc_custom_setters!(
        self.with_collider, add_collider, collider: &'a ColliderDesc<N> | { self.colliders.push(collider) }
    );

    desc_setters!(
//        with_status, set_status, status: BodyStatus
        with_parent_shift, set_parent_shift, parent_shift: Vector<N>
        with_body_shift, set_body_shift, body_shift: Vector<N>
        with_velocity, set_velocity, velocity: Velocity<N>
        with_local_inertia, set_local_inertia, local_inertia: Inertia<N>
        with_local_center_of_mass, set_local_center_of_mass, local_com: Point<N>
    );

    pub fn build<'w>(&self, world: &'w mut World<N>) -> &'w mut Multibody<N> {
        world.add_body(self)
    }

    pub fn build_with_parent<'w>(&self, parent: BodyPartHandle, world: &'w mut World<N>) -> Option<&'w mut MultibodyLink<N>> {
        if parent.is_ground() {
            Some(self.build(world).root_mut())
        } else {
            let (bodies, cworld) = world.bodies_mut_and_collider_world_mut();
            // FIXME: keep the Err so the user gets a more meaningful error?
            let mb = bodies.body_mut(parent.0)?.downcast_mut::<Multibody<N>>().ok()?;
            Some(self.do_build(mb, cworld, parent))
        }
    }

    fn do_build<'m>(&self, mb: &'m mut Multibody<N>, cworld: &mut ColliderWorld<N>, parent: BodyPartHandle) -> &'m mut MultibodyLink<N> {
        let ndofs = mb.status_dependent_ndofs();
        let link = mb.add_link(
            parent,
            self.joint.clone(),
            self.parent_shift,
            self.body_shift,
            self.local_inertia,
            self.local_com);

        link.velocity = self.velocity;

        let me = link.part_handle();

        for desc in &self.colliders {
            let _ = desc.build_with_infos(me, ndofs, link, cworld);
        }

        for child in &self.children {
            let _ = child.do_build(mb, cworld, me);
        }

        mb.link_mut(me.1).unwrap()
    }
}

impl<'a, N: Real> BodyDesc<N> for MultibodyDesc<'a, N> {
    type Body = Multibody<N>;

    fn build_with_handle(&self, cworld: &mut ColliderWorld<N>, handle: BodyHandle) -> Multibody<N> {
        let mut mb = Multibody::new(handle);
        let _ = self.do_build(&mut mb, cworld, BodyPartHandle::ground());
        mb
    }
}