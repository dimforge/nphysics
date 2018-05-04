/// Physics-specific algebraic entities: velocity, forces, and inertias.

pub use self::velocity2::Velocity2;
pub use self::velocity3::Velocity3;

pub use self::force2::Force2;
pub use self::force3::Force3;

pub use self::inertia3::Inertia3;
pub use self::inertia2::Inertia2;

mod velocity2;
mod velocity3;

mod force2;
mod force3;

mod inertia2;
mod inertia3;
