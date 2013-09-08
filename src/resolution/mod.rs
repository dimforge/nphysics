// NOTE: we re-export here only the things commonly used by the user
pub use resolution::constraint::accumulated_impulse_solver::AccumulatedImpulseSolver;
pub use resolution::constraint::contact_equation::{VelocityAndPosition, Velocity, VelocityAndPositionThresold};


pub mod solver;

pub mod constraint {
    pub mod impulse_cache;
    pub mod accumulated_impulse_solver;
    pub mod projected_gauss_seidel_solver;
    pub mod velocity_constraint;
    pub mod contact_equation;
    pub mod ball_in_socket_equation;
    pub mod fixed_equation;
}
