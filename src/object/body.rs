use na::{self, DVectorSlice, DVectorSliceMut, Real};

use object::{BodyHandle, Ground, Multibody, MultibodyLinkRef, RigidBody};
use solver::IntegrationParameters;
use math::{Force, Isometry, Point, Velocity};

#[derive(Copy, Clone, PartialEq, Eq, Hash, Debug)]
pub enum BodyStatus {
    Disabled,
    Static,
    Dynamic,
    Kinematic,
}

#[derive(Copy, Clone, Debug)]
pub struct ActivationStatus<N: Real> {
    threshold: Option<N>,
    energy: N,
}

impl<N: Real> ActivationStatus<N> {
    pub fn default_threshold() -> N {
        na::convert(0.01f64)
    }

    pub fn new_active() -> Self {
        ActivationStatus {
            threshold: Some(Self::default_threshold()),
            energy: Self::default_threshold() * na::convert(4.0),
        }
    }

    pub fn new_inactive() -> Self {
        ActivationStatus {
            threshold: Some(Self::default_threshold()),
            energy: N::zero(),
        }
    }

    #[inline]
    pub fn is_active(&self) -> bool {
        !self.energy.is_zero()
    }

    #[inline]
    pub fn deactivation_threshold(&self) -> Option<N> {
        self.threshold
    }

    #[inline]
    pub fn energy(&self) -> N {
        self.energy
    }

    #[inline]
    pub fn set_energy(&mut self, energy: N) {
        self.energy = energy
    }
}

pub enum Body<'a, N: Real> {
    RigidBody(&'a RigidBody<N>),
    Multibody(&'a Multibody<N>),
    Ground(&'a Ground<N>),
}

pub enum BodyMut<'a, N: Real> {
    RigidBody(&'a mut RigidBody<N>),
    Multibody(&'a mut Multibody<N>),
    Ground(&'a mut Ground<N>),
}

pub enum BodyPart<'a, N: Real> {
    RigidBody(&'a RigidBody<N>),
    MultibodyLink(MultibodyLinkRef<'a, N>),
    Ground(&'a Ground<N>),
}

impl<'a, N: Real> Body<'a, N> {
    #[inline]
    pub fn is_ground(&self) -> bool {
        if let Body::Ground(_) = *self {
            true
        } else {
            false
        }
    }

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
}

impl<'a, N: Real> BodyMut<'a, N> {
    #[inline]
    pub fn is_ground(&self) -> bool {
        if let BodyMut::Ground(_) = *self {
            true
        } else {
            false
        }
    }

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

    #[inline]
    pub fn integrate(&mut self, params: &IntegrationParameters<N>, added_vel: &[N]) {
        match *self {
            BodyMut::RigidBody(ref mut rb) => {
                rb.integrate(params, &Velocity::from_slice(added_vel))
            }
            BodyMut::Multibody(ref mut mb) => mb.integrate(params, Some(added_vel)),
            BodyMut::Ground(_) => {}
        }
    }
}

macro_rules! dispatch(
    ($($body: ident :: $method: ident ( $($params: ident: $argty: ty),* ) -> $ret: ty;)*) => {$(
        impl<'a, N: Real> $body<'a, N> {
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
    ($($body: ident :: $method: ident ( $($params: ident: $argty: ty),* ) -> $ret: ty;)*) => {$(
        impl<'a, N: Real> $body<'a, N> {
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
    Body::handle() -> BodyHandle;
    Body::status() -> BodyStatus;
    Body::activation_status() -> &ActivationStatus<N>;
    Body::is_active() -> bool;
    Body::is_dynamic() -> bool;
    Body::is_kinematic() -> bool;
    Body::is_static() -> bool;
    Body::ndofs() -> usize;
    Body::generalized_acceleration() -> DVectorSlice<N>;
    Body::generalized_velocity() -> DVectorSlice<N>;
    Body::companion_id() -> usize;

    BodyMut::handle() -> BodyHandle;
    BodyMut::status() -> BodyStatus;
    BodyMut::activation_status() -> &ActivationStatus<N>;
    BodyMut::is_active() -> bool;
    BodyMut::is_kinematic() -> bool;
    BodyMut::is_static() -> bool;
    BodyMut::ndofs() -> usize;
    BodyMut::generalized_acceleration() -> DVectorSlice<N>;
    BodyMut::generalized_velocity() -> DVectorSlice<N>;
    BodyMut::companion_id() -> usize;
);

dispatch_mut!(
    BodyMut::set_companion_id(id: usize) -> ();
    BodyMut::generalized_velocity_mut() -> DVectorSliceMut<N>;
    // FIXME: should those directly be implemented only for bodies (to avoid duplicated code on
    // each body for activation)?
    BodyMut::activate() -> ();
    BodyMut::activate_with_energy(energy: N) -> ();
    BodyMut::deactivate() -> ();
);

impl<'a, N: Real> BodyPart<'a, N> {
    #[inline]
    pub fn is_ground(&self) -> bool {
        if let BodyPart::Ground(_) = *self {
            true
        } else {
            false
        }
    }

    #[inline]
    pub fn handle(&self) -> BodyHandle {
        match *self {
            BodyPart::RigidBody(ref rb) => rb.handle(),
            BodyPart::MultibodyLink(ref mb) => mb.handle(),
            BodyPart::Ground(ref g) => g.handle(),
        }
    }

    #[inline]
    pub fn parent_ndofs(&self) -> usize {
        match *self {
            BodyPart::RigidBody(ref rb) => rb.ndofs(),
            BodyPart::MultibodyLink(ref mb) => mb.multibody().ndofs(),
            BodyPart::Ground(ref g) => g.ndofs(),
        }
    }

    #[inline]
    pub fn parent_companion_id(&self) -> usize {
        match *self {
            BodyPart::RigidBody(ref rb) => rb.companion_id(),
            BodyPart::MultibodyLink(ref mb) => mb.multibody().companion_id(),
            BodyPart::Ground(ref g) => g.companion_id(),
        }
    }

    #[inline]
    pub fn is_active(&self) -> bool {
        match *self {
            BodyPart::RigidBody(ref rb) => rb.is_active(),
            BodyPart::MultibodyLink(ref mb) => mb.multibody().is_active(),
            BodyPart::Ground(ref g) => g.is_active(),
        }
    }

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

    // FIXME: replace this by "generalized_velocity" when we can work
    // with sparse matrices.
    #[inline]
    pub fn parent_generalized_velocity(&self) -> DVectorSlice<N> {
        match *self {
            BodyPart::RigidBody(ref rb) => rb.generalized_velocity(),
            BodyPart::MultibodyLink(ref mb) => mb.multibody().generalized_velocity(),
            BodyPart::Ground(ref g) => g.generalized_velocity(),
        }
    }

    #[inline]
    pub fn center_of_mass(&self) -> Point<N> {
        match *self {
            BodyPart::RigidBody(ref rb) => rb.center_of_mass(),
            BodyPart::MultibodyLink(ref mb) => mb.center_of_mass(),
            BodyPart::Ground(ref g) => g.center_of_mass(),
        }
    }

    #[inline]
    pub fn position(&self) -> Isometry<N> {
        match *self {
            BodyPart::RigidBody(ref rb) => rb.position(),
            BodyPart::MultibodyLink(ref mb) => mb.position(),
            BodyPart::Ground(ref g) => g.position(),
        }
    }

    #[inline]
    pub fn velocity(&self) -> Velocity<N> {
        match *self {
            BodyPart::RigidBody(ref rb) => *rb.velocity(),
            BodyPart::MultibodyLink(ref mb) => *mb.velocity(),
            BodyPart::Ground(ref g) => g.velocity(),
        }
    }

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

    #[inline]
    pub fn body_jacobian_mul_force(&self, force: &Force<N>, out: &mut [N]) {
        match *self {
            BodyPart::RigidBody(ref rb) => rb.body_jacobian_mul_force(force, out),
            BodyPart::MultibodyLink(ref mb) => mb.body_jacobian_mul_force(force, out),
            BodyPart::Ground(ref g) => g.body_jacobian_mul_force(force, out),
        }
    }

    #[inline]
    pub fn inv_mass_mul_generalized_forces(&self, out: &mut [N]) {
        match *self {
            BodyPart::RigidBody(ref rb) => rb.inv_mass_mul_generalized_forces(out),
            BodyPart::MultibodyLink(ref mb) => mb.inv_mass_mul_generalized_forces(out),
            BodyPart::Ground(ref g) => g.inv_mass_mul_generalized_forces(out),
        }
    }

    #[inline]
    pub fn inv_mass_mul_force(&self, force: &Force<N>, out: &mut [N]) {
        match *self {
            BodyPart::RigidBody(ref rb) => rb.inv_mass_mul_force(force, out),
            BodyPart::MultibodyLink(ref mb) => mb.inv_mass_mul_force(force, out),
            BodyPart::Ground(ref g) => g.inv_mass_mul_force(force, out),
        }
    }
}
