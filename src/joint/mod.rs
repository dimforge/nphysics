pub use self::joint::{build_unit_joint_constraints, unit_joint_nconstraints,
                      unit_joint_position_constraint, Joint, UnitJoint};
pub use self::free_joint::FreeJoint;
pub use self::fixed_joint::FixedJoint;
pub use self::revolute_joint::RevoluteJoint;
pub use self::prismatic_joint::PrismaticJoint;
pub use self::cartesian_joint::CartesianJoint;

#[cfg(feature = "dim3")]
pub use self::ball_joint::BallJoint;
#[cfg(feature = "dim3")]
pub use self::planar_joint::PlanarJoint;
#[cfg(feature = "dim3")]
pub use self::cylindrical_joint::CylindricalJoint;
#[cfg(feature = "dim3")]
pub use self::pin_slot_joint::PinSlotJoint;
#[cfg(feature = "dim3")]
pub use self::helical_joint::HelicalJoint;
#[cfg(feature = "dim3")]
pub use self::universal_joint::UniversalJoint;
#[cfg(feature = "dim3")]
pub use self::rectangular_joint::RectangularJoint;

pub use self::joint_motor::JointMotor;
pub use self::constraint_generator::{ConstraintGenerator, ConstraintHandle};
pub use self::fixed_constraint::FixedConstraint;
pub use self::revolute_constraint::RevoluteConstraint;
pub use self::ball_constraint::BallConstraint;

mod joint;
mod free_joint;
mod fixed_joint;
mod revolute_joint;
mod prismatic_joint;
mod cartesian_joint;

#[cfg(feature = "dim3")]
mod ball_joint;
#[cfg(feature = "dim3")]
mod planar_joint;
#[cfg(feature = "dim3")]
mod cylindrical_joint;
#[cfg(feature = "dim3")]
mod pin_slot_joint;
#[cfg(feature = "dim3")]
mod helical_joint;
#[cfg(feature = "dim3")]
mod universal_joint;
#[cfg(feature = "dim3")]
mod rectangular_joint;

mod joint_motor;
mod constraint_generator;
mod fixed_constraint;
mod revolute_constraint;
mod ball_constraint;
