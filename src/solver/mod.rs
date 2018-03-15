//! Constraint solver.

pub use self::moreau_jean_solver::MoreauJeanSolver;
pub use self::constraint::{BilateralConstraint, BilateralGroundConstraint, ConstraintGeometry,
                           ImpulseLimits, UnilateralConstraint, UnilateralGroundConstraint};
pub use self::nonlinear_constraint::{GenericNonlinearConstraint,
                                     MultibodyJointLimitsNonlinearConstraintGenerator,
                                     NonlinearConstraintGenerator, NonlinearUnilateralConstraint};
pub use self::contact_model::ContactModel;
pub use self::signorini_coulomb_pyramid_model::SignoriniCoulombPyramidModel;
pub use self::signorini_model::SignoriniModel;
pub use self::integration_parameters::IntegrationParameters;
pub use self::helper::ForceDirection;
pub use self::impulse_cache::ImpulseCache;
pub use self::constraint_set::ConstraintSet;
pub use self::sor_prox::SORProx;
pub use self::nonlinear_sor_prox::NonlinearSORProx;

pub mod helper;
mod impulse_cache;
mod moreau_jean_solver;
mod integration_parameters;
mod contact_model;
mod signorini_coulomb_pyramid_model;
mod signorini_model;
mod constraint;
mod nonlinear_constraint;
mod constraint_set;
mod sor_prox;
mod nonlinear_sor_prox;
