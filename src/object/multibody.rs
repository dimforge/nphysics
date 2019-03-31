use std::ops::MulAssign;
use std::any::Any;

use ncollide::shape::DeformationsType;
use crate::joint::Joint;
use crate::math::{
    AngularDim, Dim, Force, Inertia, Isometry, Jacobian, Point, SpatialMatrix,
    Vector, Velocity, DIM, Translation, ForceType
};
use na::{self, DMatrix, DVector, DVectorSlice, DVectorSliceMut, Dynamic, MatrixMN, RealField, LU};
use crate::object::{
    ActivationStatus, BodyPartHandle, BodyStatus, MultibodyLink, BodyUpdateStatus,
    MultibodyLinkVec, Body, BodyPart, BodyHandle, ColliderDesc, BodyDesc
};
use crate::solver::{ConstraintSet, IntegrationParameters, ForceDirection, SORProx, NonlinearSORProx};
use crate::world::{World, ColliderWorld};
use crate::utils::{GeneralizedCross, IndexMut2};

/// An articulated body simulated using the reduced-coordinates approach.
pub struct Multibody<N: RealField> {
    name: String,
    handle: BodyHandle,
    rbs: MultibodyLinkVec<N>,
    velocities: DVector<N>,
    damping: DVector<N>,
    accelerations: DVector<N>,
    forces: DVector<N>,
    impulses: DVector<N>,
    body_jacobians: Vec<Jacobian<N>>,
    // FIXME: use sparse matrices.
    augmented_mass: DMatrix<N>,
    inv_augmented_mass: LU<N, Dynamic, Dynamic>,
    status: BodyStatus,
    gravity_enabled: bool,
    update_status: BodyUpdateStatus,
    activation: ActivationStatus<N>,
    ndofs: usize,
    companion_id: usize,
    user_data: Option<Box<Any + Send + Sync>>,

    /*
     * Workspaces.
     */
    workspace: MultibodyWorkspace<N>,
    coriolis_v: Vec<MatrixMN<N, Dim, Dynamic>>,
    coriolis_w: Vec<MatrixMN<N, AngularDim, Dynamic>>,
    i_coriolis_dt: Jacobian<N>,

    /*
     * Constraint resolution data.
     * FIXME: we should void explicitly generating those constraints by
     * just iterating on all joints at each step of the resolution.
     */
    solver_workspace: Option<SolverWorkspace<N>>
}

impl<N: RealField> Multibody<N> {
    /// Creates a new multibody with no link.
    fn new(handle: BodyHandle) -> Self {
        Multibody {
            name: String::new(),
            handle,
            rbs: MultibodyLinkVec(Vec::new()),
            velocities: DVector::zeros(0),
            forces: DVector::zeros(0),
            damping: DVector::zeros(0),
            accelerations: DVector::zeros(0),
            impulses: DVector::zeros(0),
            body_jacobians: Vec::new(),
            augmented_mass: DMatrix::zeros(0, 0),
            inv_augmented_mass: LU::new(DMatrix::zeros(0, 0)),
            status: BodyStatus::Dynamic,
            update_status: BodyUpdateStatus::all(),
            gravity_enabled: true,
            activation: ActivationStatus::new_active(),
            ndofs: 0,
            companion_id: 0,
            workspace: MultibodyWorkspace::new(),
            coriolis_v: Vec::new(),
            coriolis_w: Vec::new(),
            i_coriolis_dt: Jacobian::zeros(0),
            solver_workspace: Some(SolverWorkspace::new()),
            user_data: None
        }
    }

    user_data_accessors!();

    /// The first link of this multibody.
    #[inline]
    pub fn root(&self) -> &MultibodyLink<N> {
        &self.rbs[0]
    }

    /// Mutable reference to the first link of this multibody.
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

    /// The links of this multibody with the given `name`.
    pub fn links_with_name<'a>(&'a self, name: &'a str) -> impl Iterator<Item = &'a MultibodyLink<N>> {
        self.rbs.iter().filter(move |l| l.name == name)
    }

    /// Iterator through all the links of this multibody.
    ///
    /// All link are guaranteed to be yielded before its descendant.
    pub fn links(&self) -> impl Iterator<Item = &MultibodyLink<N>> {
        self.rbs.iter()
    }

    /// The vector of damping applied to this multibody.
    #[inline]
    pub fn damping(&self) -> &DVector<N> {
        &self.damping
    }

    /// Mutable vector of damping applied to this multibody.
    #[inline]
    pub fn damping_mut(&mut self) -> &mut DVector<N> {
        &mut self.damping
    }


    /// Set the mass of the specified link.
    #[inline]
    pub fn set_link_mass(&mut self, link_id: usize, mass: N) {
        self.update_status.set_local_inertia_changed(true);
        self.rbs[link_id].local_inertia.linear = mass;
    }

    /// Set the angular inertia of the specified linked, expressed in its local space.
    #[inline]
    #[cfg(feature = "dim3")]
    pub fn set_link_angular_inertia(&mut self, link_id: usize, angular_inertia: na::Matrix3<N>) {
        self.update_status.set_local_inertia_changed(true);
        self.rbs[link_id].local_inertia.angular = angular_inertia;
    }

    /// Set the angular inertia of the specified linked, expressed in its local space.
    #[inline]
    #[cfg(feature = "dim2")]
    pub fn set_link_angular_inertia(&mut self, link_id: usize, angular_inertia: N) {
        self.update_status.set_local_inertia_changed(true);
        self.rbs[link_id].local_inertia.angular = angular_inertia;
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
        dof.default_damping(&mut self.damping.rows_mut(assembly_id, ndofs));

        /*
         * Create the multibody.
         */
        dof.update_jacobians(&body_shift, &self.velocities.as_slice()[assembly_id..]);
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
        self.workspace.resize(self.rbs.len(), self.ndofs);

        &mut self.rbs[internal_id]
    }

    fn grow_buffers(&mut self, ndofs: usize, nimpulses: usize) {
        let len = self.velocities.len();
        self.velocities.resize_vertically_mut(len + ndofs, N::zero());
        self.forces.resize_vertically_mut(len + ndofs, N::zero());
        self.damping.resize_vertically_mut(len + ndofs, N::zero());
        self.accelerations.resize_vertically_mut(len + ndofs, N::zero());
        self.body_jacobians.push(Jacobian::zeros(0));

        let len = self.impulses.len();
        self.impulses.resize_vertically_mut(len + nimpulses, N::zero());
    }

    fn update_acceleration(&mut self, gravity: &Vector<N>) {
        if self.status != BodyStatus::Dynamic {
            return;
        }

        self.accelerations.fill(N::zero());

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

                let gravity_force = if self.gravity_enabled {
                    gravity * rb.inertia.mass()
                } else {
                    Vector::zeros()
                };

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
                self.accelerations.gemv_tr(
                    N::one(),
                    &self.body_jacobians[i],
                    external_forces.as_vector(),
                    N::one(),
                );
            }
        }

        self.accelerations.axpy(N::one(), &self.forces, N::one());
        self.accelerations.cmpy(-N::one(), &self.damping, &self.velocities, N::one());

        assert!(self.inv_augmented_mass.solve_mut(&mut self.accelerations));
    }

    /// Computes the constant terms of the dynamics.
    fn update_dynamics(&mut self, dt: N) {
        if !self.update_status.inertia_needs_update() {
            return;
        }

        /*
         * Compute velocities.
         * NOTE: this is needed for kinematic bodies too.
         */
        let rb = &mut self.rbs[0];
        let velocity_wrt_joint = rb
            .dof
            .jacobian_mul_coordinates(&self.velocities.as_slice()[rb.assembly_id..]);
        let velocity_dot_wrt_joint = rb
            .dof
            .jacobian_dot_mul_coordinates(&self.velocities.as_slice()[rb.assembly_id..]);

        rb.velocity_dot_wrt_joint = velocity_dot_wrt_joint;
        rb.velocity_wrt_joint = velocity_wrt_joint;
        rb.velocity = rb.velocity_wrt_joint;

        for i in 1..self.rbs.len() {
            let (rb, parent_rb) = self.rbs.get_mut_with_parent(i);

            let velocity_wrt_joint = rb
                .dof
                .jacobian_mul_coordinates(&self.velocities.as_slice()[rb.assembly_id..]);
            let velocity_dot_wrt_joint = rb
                .dof
                .jacobian_dot_mul_coordinates(&self.velocities.as_slice()[rb.assembly_id..]);

            rb.velocity_dot_wrt_joint =
                velocity_dot_wrt_joint.transformed(&parent_rb.local_to_world);
            rb.velocity_wrt_joint = velocity_wrt_joint.transformed(&parent_rb.local_to_world);
            rb.velocity = parent_rb.velocity + rb.velocity_wrt_joint;

            let shift = rb.center_of_mass() - parent_rb.center_of_mass();
            rb.velocity.linear += parent_rb.velocity.angular_vector().gcross(&shift);
        }

        // We don't need to update the inertia properties if we
        // have a kinematic body.
        if self.status != BodyStatus::Dynamic {
            return;
        }

        if !self.is_active() {
            self.activate();
        }

        /*
         * Update augmented mass matrix.
         */
        self.update_inertias(dt);
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

    fn update_inertias(&mut self, dt: N) {
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

                    augmented_inertia.angular += gyroscopic_matrix * dt;
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
                    &self.velocities.as_slice()[rb.assembly_id..],
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
                i_coriolis_dt_v *= rb.inertia.linear * dt;
            }

            {
                // FIXME: in 2D this is just an axpy.
                let mut i_coriolis_dt_w = self.i_coriolis_dt.fixed_rows_mut::<AngularDim>(DIM);
                i_coriolis_dt_w.gemm(
                    dt,
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
            self.augmented_mass[(i, i)] += self.damping[i] * dt;
        }

        // FIXME: avoid allocation inside LU at each timestep.
        self.inv_augmented_mass = LU::new(self.augmented_mass.clone());
    }

    /// The generalized velocity at the joint of the given link.
    #[inline]
    pub fn joint_velocity(&self, link: &MultibodyLink<N>) -> DVectorSlice<N> {
        let ndofs = link.dof.ndofs();
        DVectorSlice::from_slice(
            &self.velocities.as_slice()[link.assembly_id..link.assembly_id + ndofs],
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

        DVectorSliceMut::from_slice(&mut self.velocities.as_mut_slice()[i..i + ndofs], ndofs)
    }

    /// The generalized forces applied to this multibody at the next timestep.
    #[inline]
    pub fn generalized_force(&self) -> &DVector<N> {
        &self.forces
    }

    /// Mutable reference to the generalized forces applied to this multibody at the next timestep.
    #[inline]
    pub fn generalized_force_mut(&mut self) -> &mut DVector<N> {
        &mut self.forces
    }

    #[inline]
    pub(crate) fn impulses(&self) -> &[N] {
        self.impulses.as_slice()
    }
}

/// A temporary workspace for various updates of the multibody.
struct MultibodyWorkspace<N: RealField> {
    accs: Vec<Velocity<N>>,
    ndofs_vec: DVector<N>,
}

impl<N: RealField> MultibodyWorkspace<N> {
    /// Create an empty workspace.
    pub fn new() -> Self {
        MultibodyWorkspace {
            accs: Vec::new(),
            ndofs_vec: DVector::zeros(0)
        }
    }

    /// Resize the workspace so it is enough for `nlinks` links.
    pub fn resize(&mut self, nlinks: usize, ndofs: usize) {
        self.accs.resize(nlinks, Velocity::zero());
        self.ndofs_vec = DVector::zeros(ndofs)

    }
}

struct SolverWorkspace<N: RealField> {
    jacobians: DVector<N>,
    constraints: ConstraintSet<N>,
}

impl<N: RealField> SolverWorkspace<N> {
    pub fn new() -> Self {
        SolverWorkspace {
            jacobians: DVector::zeros(0),
            constraints: ConstraintSet::new(),
        }
    }

    pub fn resize(&mut self, nconstraints: usize, ndofs: usize) {
        let j_len = nconstraints * ndofs * 2;

        if self.jacobians.len() != j_len {
            self.jacobians = DVector::zeros(j_len);
        }
    }
}

impl<N: RealField> Body<N> for Multibody<N> {
    #[inline]
    fn name(&self) -> &str {
        &self.name
    }

    #[inline]
    fn set_name(&mut self, name: String) {
        self.name = name
    }

    #[inline]
    fn part(&self, id: usize) -> Option<&BodyPart<N>> {
        self.link(id).map(|l| l as &BodyPart<N>)
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
            rb.dof.integrate(params, &self.velocities.as_slice()[rb.assembly_id..])
        }
    }

    fn apply_displacement(&mut self, disp: &[N]) {
        self.update_status.set_position_changed(true);

        for rb in self.rbs.iter_mut() {
            rb.dof.apply_displacement(&disp[rb.assembly_id..])
        }

        self.update_kinematics();
    }

    fn clear_forces(&mut self) {
        self.forces.fill(N::zero())
    }

    fn update_kinematics(&mut self) {
        if !self.update_status.position_changed() {
            return;
        }

        // Special case for the root, which has no parent.
        {
            let rb = &mut self.rbs[0];
            rb.dof.update_jacobians(&rb.body_shift, &self.velocities.as_slice());
            rb.local_to_parent = rb.dof.body_to_parent(&rb.parent_shift, &rb.body_shift);
            rb.local_to_world = rb.local_to_parent;
            rb.com = rb.local_to_world * rb.local_com;
        }

        // Handle the children. They all have a parent within this multibody.
        for i in 1..self.rbs.len() {
            let (rb, parent_rb) = self.rbs.get_mut_with_parent(i);

            rb.dof
                .update_jacobians(&rb.body_shift, &self.velocities.as_slice()[rb.assembly_id..]);
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

    fn update_dynamics(&mut self, dt: N) {
        self.update_dynamics(dt)
    }

    fn update_acceleration(&mut self, gravity: &Vector<N>, _: &IntegrationParameters<N>) {
        self.update_acceleration(gravity)
    }

    #[inline]
    fn generalized_acceleration(&self) -> DVectorSlice<N> {
        self.accelerations.rows(0, self.ndofs)
    }

    #[inline]
    fn generalized_velocity(&self) -> DVectorSlice<N> {
        self.velocities.rows(0, self.ndofs)
    }

    #[inline]
    fn generalized_velocity_mut(&mut self) -> DVectorSliceMut<N> {
        self.update_status.set_velocity_changed(true);
        self.velocities.rows_mut(0, self.ndofs)
    }

    #[inline]
    fn gravity_enabled(&self) -> bool {
        self.gravity_enabled
    }

    #[inline]
    fn enable_gravity(&mut self, enabled: bool) {
        self.gravity_enabled = enabled
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
        self.update_status.clear();
        self.activation.set_energy(N::zero());
        self.velocities.fill(N::zero());
    }

    #[inline]
    fn set_deactivation_threshold(&mut self, threshold: Option<N>) {
        self.activation.set_deactivation_threshold(threshold)
    }

    #[inline]
    fn set_status(&mut self, status: BodyStatus) {
        self.update_status.set_status_changed(true);
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
        link.local_to_world * Translation::from(point.coords)
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
        let pos = point - link.com.coords;
        let force = force_dir.at_point(&pos);

        match self.status() {
            BodyStatus::Dynamic => {
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
                    *out_vel += force.as_vector().dot(&link.velocity.as_vector())
                }
            },
            BodyStatus::Static | BodyStatus::Disabled => {}
        }
    }

    #[inline]
    fn has_active_internal_constraints(&mut self) -> bool {
        self.links().any(|link| link.joint().num_velocity_constraints() != 0)
    }

    #[inline]
    fn setup_internal_velocity_constraints(&mut self, ext_vels: &DVectorSlice<N>, params: &IntegrationParameters<N>) {
        let mut ground_j_id = 0;
        let mut workspace = self.solver_workspace.take().unwrap();

        /*
         * Cache impulses from the last timestep for warmstarting.
         */
        // FIXME: should this be another pass of the solver (happening after all the resolution completed).
        for c in &workspace.constraints.velocity.unilateral_ground {
            self.impulses[c.impulse_id] = c.impulse;
        }

        for c in &workspace.constraints.velocity.bilateral_ground {
            self.impulses[c.impulse_id] = c.impulse;
        }

        workspace.constraints.clear();

        /*
         * Setup the constraints.
         */
        let nconstraints = self.rbs
            .iter()
            .map(|l| l.joint().num_velocity_constraints())
            .sum();

        workspace.resize(nconstraints, self.ndofs);

        for link in self.rbs.iter() {
            link.joint().velocity_constraints(
                params,
                self,
                &link,
                0,
                0,
                ext_vels.as_slice(),
                &mut ground_j_id,
                workspace.jacobians.as_mut_slice(),
                &mut workspace.constraints,
            );
        }

        self.solver_workspace = Some(workspace);
    }

    #[inline]
    fn warmstart_internal_velocity_constraints(&mut self, dvels: &mut DVectorSliceMut<N>) {
        let workspace = self.solver_workspace.as_mut().unwrap();
        for c in &mut workspace.constraints.velocity.unilateral_ground {
            let dim = Dynamic::new(c.ndofs);
            SORProx::warmstart_unilateral_ground(c, workspace.jacobians.as_slice(), dvels, dim)
        }

        for c in &mut workspace.constraints.velocity.bilateral_ground {
            let dim = Dynamic::new(c.ndofs);
            SORProx::warmstart_bilateral_ground(c, workspace.jacobians.as_slice(), dvels, dim)
        }
    }

    #[inline]
    fn step_solve_internal_velocity_constraints(&mut self, dvels: &mut DVectorSliceMut<N>) {
        let workspace = self.solver_workspace.as_mut().unwrap();
        for c in &mut workspace.constraints.velocity.unilateral_ground {
            let dim = Dynamic::new(c.ndofs);
            SORProx::solve_unilateral_ground(c, workspace.jacobians.as_slice(), dvels, dim)
        }

        for c in &mut workspace.constraints.velocity.bilateral_ground {
            let dim = Dynamic::new(c.ndofs);
            SORProx::solve_bilateral_ground(c, &[], workspace.jacobians.as_slice(), dvels, dim)
        }
    }

    #[inline]
    fn step_solve_internal_position_constraints(&mut self, params: &IntegrationParameters<N>) {
        // FIXME: this `.take()` trick is ugly.
        // We should not pass a reference to the multibody to the link position constraint method.
        let mut workspace = self.solver_workspace.take().unwrap();
        let jacobians = &mut workspace.jacobians;

        for i in 0..self.rbs.len() {
            for j in 0..self.rbs[i].joint().num_position_constraints() {
                let link = &self.rbs[i];
                // FIXME: should each link directly solve the constraint internally
                // instead of having to return a GenericNonlinearConstraint struct
                // every time.
                let c = link
                    .joint()
                    .position_constraint(j, self, link, 0, jacobians.as_mut_slice());

                if let Some(c) = c {
                    // FIXME: the following has been copy-pasted from the NonlinearSORProx.
                    // We should refactor the code better.
                    let rhs = NonlinearSORProx::clamp_rhs(c.rhs, c.is_angular, params);

                    if rhs < N::zero() {
                        let impulse = -rhs * c.r;
                        jacobians.rows_mut(c.wj_id1, c.dim1).mul_assign(impulse);

                        // FIXME: we should not use apply_displacement to avoid
                        // performing the .update_kinematic().
                        self.apply_displacement(jacobians.rows(c.wj_id1, c.dim1).as_slice(), );
                    }
                }
            }
        }
        self.solver_workspace = Some(workspace);
        self.update_kinematics();
    }

    #[inline]
    fn add_local_inertia_and_com(&mut self, part_id: usize, com: Point<N>, inertia: Inertia<N>) {
        self.update_status.set_local_inertia_changed(true);
        let mut link = &mut self.rbs[part_id];
        // Update center of mass.
        if !link.inertia.linear.is_zero() {
            let mass_sum = link.inertia.linear + inertia.linear;
            link.local_com = (link.local_com * link.inertia.linear + com.coords * inertia.linear) / mass_sum;
            link.com = link.local_to_world * link.local_com;
        }

        // Update inertia.
        link.local_inertia += inertia;
    }

    fn apply_force(&mut self, part_id: usize, force: &Force<N>, force_type: ForceType, auto_wake_up: bool) {
        if self.status != BodyStatus::Dynamic {
            return;
        }

        if auto_wake_up {
            self.activate()
        }

        match force_type {
            ForceType::Force => {
                self.forces.gemv_tr(N::one(), &self.body_jacobians[part_id], force.as_vector(), N::one())
            }
            ForceType::Impulse => {
                self.update_status.set_velocity_changed(true);
                let dvel = &mut self.workspace.ndofs_vec;
                dvel.gemv_tr(N::one(), &self.body_jacobians[part_id], force.as_vector(), N::zero());
                let _ = self.inv_augmented_mass.solve_mut(dvel);
                self.velocities.axpy(N::one(), dvel, N::one());
            }
            ForceType::AccelerationChange => {
                let force = self.rbs[part_id].inertia * *force;
                self.forces.gemv_tr(N::one(), &self.body_jacobians[part_id], force.as_vector(), N::one());
            }
            ForceType::VelocityChange => {
                self.update_status.set_velocity_changed(true);
                self.velocities.gemv_tr(N::one(), &self.body_jacobians[part_id], force.as_vector(), N::one())
            }
        }
    }

    fn apply_local_force(&mut self, part_id: usize, force: &Force<N>, force_type: ForceType, auto_wake_up: bool) {
        let world_force = force.transform_by(&self.rbs[part_id].local_to_world);
        self.apply_force(part_id, &world_force, force_type, auto_wake_up);
    }

    fn apply_force_at_point(&mut self, part_id: usize, force: &Vector<N>, point: &Point<N>, force_type: ForceType, auto_wake_up: bool) {
        let force = Force::linear_at_point(*force, &(point - self.rbs[part_id].com.coords));
        self.apply_force(part_id, &force, force_type, auto_wake_up)
    }

    fn apply_local_force_at_point(&mut self, part_id: usize, force: &Vector<N>, point: &Point<N>, force_type: ForceType, auto_wake_up: bool) {
        self.apply_force_at_point(part_id, &(self.rbs[part_id].local_to_world * force), point, force_type, auto_wake_up)
    }

    fn apply_force_at_local_point(&mut self, part_id: usize, force: &Vector<N>, point: &Point<N>, force_type: ForceType, auto_wake_up: bool) {
        self.apply_force_at_point(part_id, force, &(self.rbs[part_id].local_to_world * point), force_type, auto_wake_up)
    }

    fn apply_local_force_at_local_point(&mut self, part_id: usize, force: &Vector<N>, point: &Point<N>, force_type: ForceType, auto_wake_up: bool) {
        self.apply_force_at_point(
            part_id,
            &(self.rbs[part_id].local_to_world * force),
            &(self.rbs[part_id].local_to_world * point),
            force_type,
            auto_wake_up)
    }
}


/// A multibody builder.
pub struct MultibodyDesc<'a, N: RealField> {
    name: String,
    children: Vec<MultibodyDesc<'a, N>>,
    joint: Box<Joint<N>>,
    velocity: Velocity<N>,
    local_inertia: Inertia<N>,
    local_center_of_mass: Point<N>,
    colliders: Vec<&'a ColliderDesc<N>>,
    body_shift: Vector<N>,
    parent_shift: Vector<N>
}

impl<'a, N: RealField> MultibodyDesc<'a, N> {
    /// Initialize a multibody builder with one link with one joint.
    pub fn new<J: Joint<N>>(joint: J) -> Self {
        MultibodyDesc {
            name: String::new(),
            joint: Box::new(joint),
            children: Vec::new(),
            velocity: Velocity::zero(),
            local_inertia: Inertia::zero(),
            local_center_of_mass: Point::origin(),
            colliders: Vec::new(),
            body_shift: Vector::zeros(),
            parent_shift: Vector::zeros()
        }
    }

    /// Add a children link to the multibody link represented by `self`.
    pub fn add_child<J: Joint<N>>(&mut self, joint: J) -> &mut MultibodyDesc<'a, N> {
        let child = MultibodyDesc::new(joint);

        self.children.push(child);
        self.children.last_mut().unwrap()
    }

    /// Sets the joint of this multibody builder.
    pub fn set_joint<J: Joint<N>>(&mut self, joint: J) -> &mut Self {
        self.joint = Box::new(joint);
        self
    }

    #[cfg(feature = "dim2")]
    desc_custom_setters!(
        self.angular_inertia, set_angular_inertia, angular_inertia: N | { self.local_inertia.angular = angular_inertia }
    );

    #[cfg(feature = "dim3")]
    desc_custom_setters!(
        self.angular_inertia, set_angular_inertia, angular_inertia: na::Matrix3<N> | { self.local_inertia.angular = angular_inertia }
    );

    desc_custom_setters!(
        self.collider, add_collider, collider: &'a ColliderDesc<N> | { self.colliders.push(collider) }
        self.mass, set_mass, mass: N | { self.local_inertia.linear = mass }
    );

    desc_setters!(
//        status, set_status, status: BodyStatus
        name, set_name, name: String
        parent_shift, set_parent_shift, parent_shift: Vector<N>
        body_shift, set_body_shift, body_shift: Vector<N>
        velocity, set_velocity, velocity: Velocity<N>
        local_inertia, set_local_inertia, local_inertia: Inertia<N>
        local_center_of_mass, set_local_center_of_mass, local_center_of_mass: Point<N>
    );


    #[cfg(feature = "dim2")]
    desc_custom_getters!(
        self.get_angular_inertia: N | { self.local_inertia.angular }
    );

    #[cfg(feature = "dim3")]
    desc_custom_getters!(
        self.get_angular_inertia: &na::Matrix3<N> | { &self.local_inertia.angular }
    );

    desc_custom_getters!(
        self.get_mass: N | { self.local_inertia.linear }
        self.get_name: &str | { &self.name }
    );

    desc_getters!(
        [ref] get_parent_shift -> parent_shift: Vector<N>
        [ref] get_body_shift -> body_shift: Vector<N>
        [ref] get_velocity -> velocity: Velocity<N>
        [ref] get_local_inertia -> local_inertia: Inertia<N>
        [ref] get_local_center_of_mass -> local_center_of_mass: Point<N>
    );


    /// Build into the `world` the multibody represented by `self` and its children.
    pub fn build<'w>(&self, world: &'w mut World<N>) -> &'w mut Multibody<N> {
        world.add_body(self)
    }

    /// Adds the links represented by `self` and its children to the multibody identified by `parent`.
    ///
    /// If `parent` is the ground, then a new multibody is created.
    /// If `parent` is not another multibody link, then `None` is returned.
    pub fn build_with_parent<'w>(&self, parent: BodyPartHandle, world: &'w mut World<N>) -> Option<&'w mut MultibodyLink<N>> {
        if parent.is_ground() {
            Some(self.build(world).root_mut())
        } else {
            let (bodies, cworld) = world.bodies_mut_and_collider_world_mut();
            // FIXME: keep the Err so the user gets a more meaningful error?
            let mb = bodies.body_mut(parent.0)?.downcast_mut::<Multibody<N>>()?;
            Some(self.do_build(mb, cworld, parent))
        }
    }

    fn do_build<'m>(&self, mb: &'m mut Multibody<N>, cworld: &mut ColliderWorld<N>, parent: BodyPartHandle) -> &'m mut MultibodyLink<N> {
        let link = mb.add_link(
            parent,
            self.joint.clone(),
            self.parent_shift,
            self.body_shift,
            self.local_inertia,
            self.local_center_of_mass);

        link.velocity = self.velocity;
        link.name = self.name.clone();

        let me = link.part_handle();

        for desc in &self.colliders {
            let _ = desc.build_with_infos(me, mb, cworld);
        }

        for child in &self.children {
            let _ = child.do_build(mb, cworld, me);
        }

        mb.link_mut(me.1).unwrap()
    }
}

impl<'a, N: RealField> BodyDesc<N> for MultibodyDesc<'a, N> {
    type Body = Multibody<N>;

    fn build_with_handle(&self, cworld: &mut ColliderWorld<N>, handle: BodyHandle) -> Multibody<N> {
        let mut mb = Multibody::new(handle);
        let _ = self.do_build(&mut mb, cworld, BodyPartHandle::ground());
        mb
    }
}