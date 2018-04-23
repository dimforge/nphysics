pub use self::joint::Joint;
pub use self::unit_joint::{unit_joint_num_velocity_constraints, unit_joint_position_constraint,
                           unit_joint_velocity_constraints, UnitJoint};
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
pub use self::joint_constraint::{ConstraintHandle, JointConstraint};
pub use self::fixed_constraint::FixedConstraint;
pub use self::revolute_constraint::RevoluteConstraint;
pub use self::prismatic_constraint::PrismaticConstraint;
pub use self::ball_constraint::BallConstraint;
pub use self::mouse_constraint::MouseConstraint;

mod joint;
mod unit_joint;
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
mod joint_constraint;
mod fixed_constraint;
mod revolute_constraint;
mod prismatic_constraint;
mod ball_constraint;
mod mouse_constraint;
