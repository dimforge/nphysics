//! Dynamics-specific algebraic entities: velocity, forces, and inertias.

pub use self::velocity2::Velocity2;
pub use self::velocity3::Velocity3;

pub use self::force2::Force2;
pub use self::force3::Force3;

pub use self::inertia2::Inertia2;
pub use self::inertia3::Inertia3;

/// The type of force to be applied with the `.apply_force` methods of a Body.
///
/// See the [user guide](https://www.nphysics.org/rigid_body_simulations_with_contacts/#one-time-force-application-and-impulses)
/// for details.
#[derive(Clone, Copy, Debug)]
pub enum ForceType {
    /// A regular force.
    Force,
    /// An impulsive force (delta-time is ignored).
    Impulse,
    /// A direct acceleration change (mass is ignored).
    AccelerationChange,
    /// A direct velocity change (mass and delta time are ignored).
    VelocityChange,
}

mod velocity2;
mod velocity3;

mod force2;
mod force3;

mod inertia2;
mod inertia3;
