use nalgebra::na;
use ncollide::math::{Scalar, Vect, Orientation};

#[deriving(Eq, Show, Clone)]
/// A constraint of velocity at a point of contact.
pub struct VelocityConstraint {
    /// The contact normal.
    normal:             Vect,

    /// The contact normal multiplied by the first body's inverse mass.
    weighted_normal1:   Vect,
    /// The contact normal multiplied by the second body's inverse mass.
    weighted_normal2:   Vect,

    /// The first body rotation axis.
    rot_axis1:          Orientation,
    /// The first body rotation axis multiplied by its inverse inertia.
    weighted_rot_axis1: Orientation,

    /// The second body rotation axis.
    rot_axis2:          Orientation,
    /// The second body rotation axis multiplied by its inverse inertia.
    weighted_rot_axis2: Orientation,

    /// The inverse of the sum of linear and angular inertia of both bodies.
    inv_projected_mass: Scalar,

    /// The total impulse applied.
    impulse:            Scalar,
    /// The lower bound of the impulse.
    lobound:            Scalar,
    /// The upper bound of the impulse.
    hibound:            Scalar,
    /// The target delta velocity.
    objective:          Scalar,
    /// The id of the first body.
    id1:                int,
    /// The id of the second body.
    id2:                int,
    /// The id of the friction constraint.
    friction_limit_id:  uint,
    /// The friction coefficient on this contact.
    friction_coeff:     Scalar
}

impl VelocityConstraint {
    /// Creates a new velocity constraint with all terms initialized to zero.
    pub fn new() -> VelocityConstraint {
        VelocityConstraint {
            normal:             na::zero(),

            weighted_normal1:   na::zero(),
            weighted_normal2:   na::zero(),

            rot_axis1:          na::zero(),
            weighted_rot_axis1: na::zero(),

            rot_axis2:          na::zero(),
            weighted_rot_axis2: na::zero(),

            inv_projected_mass: na::zero(),

            impulse:            na::zero(),
            hibound:            na::zero(),
            lobound:            na::zero(),
            objective:          na::zero(),
            id1:                -1,
            id2:                -1,
            friction_limit_id:  0,
            friction_coeff:     na::zero()
        }
    }
}
