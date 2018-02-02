//! Constraint solver.

pub use self::moreau_jean_solver::MoreauJeanSolver;
pub use self::velocity_constraint::{BilateralConstraint2, BilateralGroundConstraint,
                                    ConstraintGeometry, ImpulseLimits, UnilateralConstraint2,
                                    UnilateralGroundConstraint};
pub use self::contact_model::ContactModel;
pub use self::signorini_coulomb_pyramid_model::SignoriniCoulombPyramidModel;
pub use self::signorini_model::SignoriniModel;
pub use self::sor_prox::SORProx;
pub use self::integration_parameters::IntegrationParameters;
pub use self::helper::ForceDirection;

pub mod helper;
mod moreau_jean_solver;
mod sor_prox;
mod integration_parameters;
mod contact_model;
mod signorini_coulomb_pyramid_model;
mod signorini_model;
mod velocity_constraint;
