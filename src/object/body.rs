use na::{self, DVectorSlice, DVectorSliceMut, Real};

use math::{Force, Inertia, Isometry, Point, Velocity};
use object::{BodyHandle, Ground, Multibody, MultibodyLinkMut, MultibodyLinkRef, RigidBody};
use solver::IntegrationParameters;

/// The status of a body.
#[derive(Copy, Clone, PartialEq, Eq, Hash, Debug)]
pub enum BodyStatus {
    /// The body is disabled and ignored by the physics engine.
    Disabled,
    /// The body is static and thus cannot move.
    Static,
    /// The body is dynamic and thus can move and is subject to forces.
    Dynamic,
    /// The body is kinematic so its velocity is controlled by the user and it is not affected by forces and constraints.
    Kinematic,
}

/// The activation status of a body.
///
/// This controls whether a body is sleeping or not.
#[derive(Copy, Clone, Debug)]
pub struct ActivationStatus<N: Real> {
    threshold: Option<N>,
    energy: N,
}

impl<N: Real> ActivationStatus<N> {
    /// The default amount of energy bellow which a body can be put to sleep by nphysics.
    pub fn default_threshold() -> N {
        na::convert(0.01f64)
    }

    /// Create a new activation status initialised with the default activation threshold and is active.
    pub fn new_active() -> Self {
        ActivationStatus {
            threshold: Some(Self::default_threshold()),
            energy: Self::default_threshold() * na::convert(4.0),
        }
    }

    /// Create a new activation status initialised with the default activation threshold and is inactive.
    pub fn new_inactive() -> Self {
        ActivationStatus {
            threshold: Some(Self::default_threshold()),
            energy: N::zero(),
        }
    }

    /// Retuns `true` if the body is not asleep.
    #[inline]
    pub fn is_active(&self) -> bool {
        !self.energy.is_zero()
    }

    /// The threshold bellow which the body can be put to sleep.
    ///
    /// A value of `None` indicates that the body cannot sleep.
    #[inline]
    pub fn deactivation_threshold(&self) -> Option<N> {
        self.threshold
    }

    /// Set the threshold bellow which the body can be put to sleep.
    ///
    /// A value of `None` prevents the body from sleeping.
    #[inline]
    pub fn set_deactivation_threshold(&mut self, threshold: Option<N>) {
        self.threshold = threshold
    }

    /// The current energy averaged through several frames.
    #[inline]
    pub fn energy(&self) -> N {
        self.energy
    }

    /// Sets the current average energy of the body.
    #[inline]
    pub fn set_energy(&mut self, energy: N) {
        self.energy = energy
    }
}

/// A body contained by the physics world.
pub enum Body<'a, N: Real> {
    /// A rigid body.
    RigidBody(&'a RigidBody<N>),
    /// A multibody.
    Multibody(&'a Multibody<N>),
    /// The ground.
    Ground(&'a Ground<N>),
}

/// A mutable body contained by the physics world.
pub enum BodyMut<'a, N: Real> {
    /// A rigid body.
    RigidBody(&'a mut RigidBody<N>),
    /// A multibody.
    Multibody(&'a mut Multibody<N>),
    /// The ground.
    Ground(&'a mut Ground<N>),
}

/// A mutable body part contained by the physics world.
pub enum BodyPart<'a, N: Real> {
    /// A rigid body.
    RigidBody(&'a RigidBody<N>),
    /// A link of a multibody.
    MultibodyLink(MultibodyLinkRef<'a, N>),
    /// The ground.
    Ground(&'a Ground<N>),
}

/// A mutable body part contained by the physics world.
pub enum BodyPartMut<'a, N: Real> {
    /// A rigid body.
    RigidBody(&'a mut RigidBody<N>),
    /// A link of a multibody.
    MultibodyLink(MultibodyLinkMut<'a, N>),
    /// The ground.
    Ground(&'a mut Ground<N>),
}

impl<'a, N: Real> Body<'a, N> {
    /// Checks if this body identifies the ground.
    #[inline]
    pub fn is_ground(&self) -> bool {
        if let Body::Ground(_) = *self {
            true
        } else {
            false
        }
    }

    /// The number of degrees of freedom (DOF) of this body, taking its status into account.
    ///
    /// In particular, this returns 0 for any body with a status different than `BodyStatus::Dynamic`.
    #[inline]
    pub fn status_dependent_ndofs(&self) -> usize {
        match *self {
            Body::RigidBody(ref rb) => if rb.is_dynamic() {
                rb.ndofs()
            } else {
                0
            },
            Body::Multibody(ref mb) => if mb.is_dynamic() {
                mb.ndofs()
            } else {
                0
            },
            Body::Ground(_) => 0,
        }
    }

    /// The position of this body, relative to the ground.
    #[inline]
    pub fn position(&self) -> Isometry<N> {
        match self {
            Body::RigidBody(rb) => rb.position(),
            Body::Multibody(mb) => mb.position(),
            Body::Ground(g) => g.position(),
        }
    }
}

impl<'a, N: Real> BodyMut<'a, N> {
    /// Checks if this body identifies the ground.
    #[inline]
    pub fn is_ground(&self) -> bool {
        if let BodyMut::Ground(_) = *self {
            true
        } else {
            false
        }
    }

    /// The number of degrees of freedom (DOF) of this body, taking its status into account.
    ///
    /// In particular, this returns 0 for any body with a status different than `BodyStatus::Dynamic`.
    #[inline]
    pub fn status_dependent_ndofs(&self) -> usize {
        match *self {
            BodyMut::RigidBody(ref rb) => if rb.is_dynamic() {
                rb.ndofs()
            } else {
                0
            },
            BodyMut::Multibody(ref mb) => if mb.is_dynamic() {
                mb.ndofs()
            } else {
                0
            },
            BodyMut::Ground(_) => 0,
        }
    }

    /// The position of this body, relative to the ground.
    #[inline]
    pub fn position(&self) -> Isometry<N> {
        match self {
            BodyMut::RigidBody(rb) => rb.position(),
            BodyMut::Multibody(mb) => mb.position(),
            BodyMut::Ground(g) => g.position(),
        }
    }

    ///
    /// Applies a displacement to all the degrees of freedom of this body.
    #[inline]
    pub fn apply_displacement(&mut self, disp: &[N]) {
        match *self {
            BodyMut::RigidBody(ref mut rb) => rb.apply_displacement(&Velocity::from_slice(disp)),
            BodyMut::Multibody(ref mut mb) => mb.apply_displacement(disp),
            BodyMut::Ground(_) => {}
        }
    }
}

macro_rules! dispatch(
    ($($body: ident :: $method: ident ( $($params: ident: $argty: ty),* ) -> $ret: ty | $doc: expr;)*) => {$(
        impl<'a, N: Real> $body<'a, N> {
            #[doc = $doc]
            #[inline]
            pub fn $method(&self $($params: $argty),*) -> $ret {
                match *self {
                    $body::RigidBody(ref rb) => rb.$method($($params),*),
                    $body::Multibody(ref mb) => mb.$method($($params),*),
                    $body::Ground(ref g)     => g.$method($($params),*),
                }
            }
        })*
    }
);

macro_rules! dispatch_mut(
    ($($body: ident :: $method: ident ( $($params: ident: $argty: ty),* ) -> $ret: ty | $doc: expr;)*) => {$(
        impl<'a, N: Real> $body<'a, N> {
            #[doc = $doc]
            #[inline]
            pub fn $method(&mut self $(, $params: $argty)*) -> $ret {
                match *self {
                    $body::RigidBody(ref mut rb) => rb.$method($($params),*),
                    $body::Multibody(ref mut mb) => mb.$method($($params),*),
                    $body::Ground(ref mut g)     => g.$method($($params),*),
                }
            }
        })*
    }
);

dispatch!(
    Body::handle() -> BodyHandle | "The handle of this body.";
    Body::status() -> BodyStatus | "The status of this body.";
    Body::activation_status() -> &ActivationStatus<N> | "Informations regarding activation and deactivation (sleeping) of this body.";
    Body::is_active() -> bool | "Check if this body is active.";
    Body::is_dynamic() -> bool | "Whether or not the status of this body is dynamic.";
    Body::is_kinematic() -> bool | "Whether or not the status of this body is kinematic.";
    Body::is_static() -> bool | "Whether or not the status of this body is static.";
    Body::ndofs() -> usize | "The number of degrees of freedom of this body.";
    Body::generalized_acceleration() -> DVectorSlice<N> | "The generalized accelerations at each degree of freedom of this body.";
    Body::generalized_velocity() -> DVectorSlice<N> | "The generalized velocities of this body.";
    Body::companion_id() -> usize | "The companion ID of this body.";

    BodyMut::handle() -> BodyHandle | "The handle of this body.";
    BodyMut::status() -> BodyStatus | "The status of this body.";
    BodyMut::activation_status() -> &ActivationStatus<N> | "Informations regarding activation and deactivation (sleeping) of this body.";
    BodyMut::is_active() -> bool | "Check if this body is active.";
    BodyMut::is_kinematic() -> bool | "Whether or not the status of this body is kinematic.";
    BodyMut::is_static() -> bool | "Whether or not the status of this body is static.";
    BodyMut::ndofs() -> usize | "The number of degrees of freedom of this body.";
    BodyMut::generalized_acceleration() -> DVectorSlice<N> | "The generalized accelerations at each degree of freedom of this body.";
    BodyMut::generalized_velocity() -> DVectorSlice<N> | "The generalized velocities of this body.";
    BodyMut::companion_id() -> usize | "The companion ID of this body.";
);

dispatch_mut!(
    BodyMut::set_companion_id(id: usize) -> () | "Set the companion ID of this body (may be reinitialized by nphysics).";
    BodyMut::generalized_velocity_mut() -> DVectorSliceMut<N> | "The mutable generalized velocities of this body.";
    BodyMut::integrate(params: &IntegrationParameters<N>) -> () | "Integrate the position of this body.";
    // FIXME: should those directly be implemented only for bodies (to avoid duplicated code on
    // each body for activation)?
    BodyMut::activate() -> () | "Force the activation of this body.";
    BodyMut::activate_with_energy(energy: N) -> () | "Force the activation of this body with the given level of energy.";
    BodyMut::deactivate() -> () | "Put this body to sleep.";
);

impl<'a, N: Real> BodyPart<'a, N> {
    /// Returns `true` if this body is the ground.
    #[inline]
    pub fn is_ground(&self) -> bool {
        if let BodyPart::Ground(_) = *self {
            true
        } else {
            false
        }
    }

    /// The handle of this body.
    #[inline]
    pub fn handle(&self) -> BodyHandle {
        match *self {
            BodyPart::RigidBody(ref rb) => rb.handle(),
            BodyPart::MultibodyLink(ref mb) => mb.handle(),
            BodyPart::Ground(ref g) => g.handle(),
        }
    }

    /// The number of degrees of freedom of the body containing this body part.
    #[inline]
    pub fn parent_ndofs(&self) -> usize {
        match *self {
            BodyPart::RigidBody(ref rb) => rb.ndofs(),
            BodyPart::MultibodyLink(ref mb) => mb.multibody().ndofs(),
            BodyPart::Ground(ref g) => g.ndofs(),
        }
    }

    /// The companion ID of the parent of this body part.
    #[inline]
    pub fn parent_companion_id(&self) -> usize {
        match *self {
            BodyPart::RigidBody(ref rb) => rb.companion_id(),
            BodyPart::MultibodyLink(ref mb) => mb.multibody().companion_id(),
            BodyPart::Ground(ref g) => g.companion_id(),
        }
    }

    /// Check if this body part is active.
    #[inline]
    pub fn is_active(&self) -> bool {
        match *self {
            BodyPart::RigidBody(ref rb) => rb.is_active(),
            BodyPart::MultibodyLink(ref mb) => mb.multibody().is_active(),
            BodyPart::Ground(ref g) => g.is_active(),
        }
    }

    /// The number of degrees of freedom (DOF) of the body containing this body part, taking its status into account.
    ///
    /// In particular, this returns 0 for any body with a status different than `BodyStatus::Dynamic`.
    #[inline]
    pub fn status_dependent_parent_ndofs(&self) -> usize {
        match *self {
            BodyPart::RigidBody(ref rb) => if rb.is_dynamic() {
                rb.ndofs()
            } else {
                0
            },
            BodyPart::MultibodyLink(ref mb) => if mb.multibody().is_dynamic() {
                mb.multibody().ndofs()
            } else {
                0
            },
            BodyPart::Ground(_) => 0,
        }
    }

    /// The generalized velocities of the body containing this body part.
    #[inline]
    pub fn parent_generalized_velocity(&self) -> DVectorSlice<N> {
        match *self {
            BodyPart::RigidBody(ref rb) => rb.generalized_velocity(),
            BodyPart::MultibodyLink(ref mb) => mb.multibody().generalized_velocity(),
            BodyPart::Ground(ref g) => g.generalized_velocity(),
        }
    }

    /// The center of mass of this body part.
    #[inline]
    pub fn center_of_mass(&self) -> Point<N> {
        match *self {
            BodyPart::RigidBody(ref rb) => rb.center_of_mass(),
            BodyPart::MultibodyLink(ref mb) => mb.center_of_mass(),
            BodyPart::Ground(ref g) => g.center_of_mass(),
        }
    }

    /// The position of this body part wrt. the ground.
    #[inline]
    pub fn position(&self) -> Isometry<N> {
        match *self {
            BodyPart::RigidBody(ref rb) => rb.position(),
            BodyPart::MultibodyLink(ref mb) => mb.position(),
            BodyPart::Ground(ref g) => g.position(),
        }
    }

    /// The velocity of this body part.
    #[inline]
    pub fn velocity(&self) -> Velocity<N> {
        match *self {
            BodyPart::RigidBody(ref rb) => *rb.velocity(),
            BodyPart::MultibodyLink(ref mb) => *mb.velocity(),
            BodyPart::Ground(ref g) => g.velocity(),
        }
    }

    /// The world-space inertia of this body part.
    #[inline]
    pub fn inertia(&self) -> Inertia<N> {
        match *self {
            BodyPart::RigidBody(ref rb) => *rb.inertia(),
            BodyPart::MultibodyLink(ref mb) => *mb.inertia(),
            BodyPart::Ground(ref g) => g.inertia(),
        }
    }

    /// The local-space inertia of this body part.
    #[inline]
    pub fn local_inertia(&self) -> Inertia<N> {
        match *self {
            BodyPart::RigidBody(ref rb) => *rb.local_inertia(),
            BodyPart::MultibodyLink(ref mb) => *mb.local_inertia(),
            BodyPart::Ground(ref g) => g.local_inertia(),
        }
    }

    /// The velocity of this body part, taking it status into account.
    ///
    /// This always return zero if the body part is static.
    #[inline]
    pub fn status_dependent_velocity(&self) -> Velocity<N> {
        match *self {
            BodyPart::RigidBody(ref rb) => if !rb.is_static() {
                *rb.velocity()
            } else {
                Velocity::zero()
            },
            BodyPart::MultibodyLink(ref mb) => if mb.multibody().is_static() {
                *mb.velocity()
            } else {
                Velocity::zero()
            },
            BodyPart::Ground(_) => Velocity::zero(),
        }
    }

    /// Convert a force applied to the center of mass of this body part into generalized force.
    #[inline]
    pub fn body_jacobian_mul_force(&self, force: &Force<N>, out: &mut [N]) {
        match *self {
            BodyPart::RigidBody(ref rb) => rb.body_jacobian_mul_force(force, out),
            BodyPart::MultibodyLink(ref mb) => mb.body_jacobian_mul_force(force, out),
            BodyPart::Ground(ref g) => g.body_jacobian_mul_force(force, out),
        }
    }

    /// Convert generalized forces applied to this body part into generalized accelerations.
    #[inline]
    pub fn inv_mass_mul_generalized_forces(&self, out: &mut [N]) {
        match *self {
            BodyPart::RigidBody(ref rb) => rb.inv_mass_mul_generalized_forces(out),
            BodyPart::MultibodyLink(ref mb) => mb.inv_mass_mul_generalized_forces(out),
            BodyPart::Ground(ref g) => g.inv_mass_mul_generalized_forces(out),
        }
    }

    /// Convert a force applied to this body part's center of mass into generalized accelerations.
    #[inline]
    pub fn inv_mass_mul_force(&self, force: &Force<N>, out: &mut [N]) {
        match *self {
            BodyPart::RigidBody(ref rb) => rb.inv_mass_mul_force(force, out),
            BodyPart::MultibodyLink(ref mb) => mb.inv_mass_mul_force(force, out),
            BodyPart::Ground(ref g) => g.inv_mass_mul_force(force, out),
        }
    }
}

impl<'a, N: Real> BodyPartMut<'a, N> {
    /// Retrieves an immutable reference to this body part.
    #[inline]
    pub fn as_ref<'b>(&'b self) -> BodyPart<'b, N> {
        match *self {
            BodyPartMut::RigidBody(ref rb) => BodyPart::RigidBody(rb),
            BodyPartMut::MultibodyLink(ref mb) => BodyPart::MultibodyLink(mb.as_ref()),
            BodyPartMut::Ground(ref g) => BodyPart::Ground(g),
        }
    }

    /// Apply a force to this body part at the next frame.
    #[inline]
    pub fn apply_force(&mut self, force: &Force<N>) {
        match *self {
            BodyPartMut::RigidBody(ref mut rb) => rb.apply_force(force),
            BodyPartMut::MultibodyLink(ref mut mb) => mb.apply_force(force),
            BodyPartMut::Ground(ref mut g) => g.apply_force(force),
        }
    }
}
