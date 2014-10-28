//! Constraint resolution.

pub use resolution::solver::Solver;
pub use resolution::constraint::accumulated_impulse_solver::AccumulatedImpulseSolver;
pub use resolution::constraint::contact_equation::{VelocityAndPosition, Velocity,
                                                   VelocityAndPositionThresold, CorrectionMode};
pub use resolution::constraint::projected_gauss_seidel_solver::{Velocities, projected_gauss_seidel_solve};
pub use resolution::constraint::impulse_cache::{ImpulseCache, ContactIdentifier};
pub use resolution::constraint::velocity_constraint::VelocityConstraint;


// XXX: `pub` due to rust#18241
#[allow(missing_doc)]
pub mod solver;

mod constraint {
    pub mod impulse_cache;
    pub mod accumulated_impulse_solver;
    pub mod projected_gauss_seidel_solver;
    pub mod velocity_constraint;
    pub mod contact_equation;
    pub mod ball_in_socket_equation;
    pub mod fixed_equation;
}
