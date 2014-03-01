use nalgebra::na;
use ncollide::math::{N, LV, AV};

#[deriving(Eq, Show, Clone)]
/// A constraint of velocity at a point of contact.
pub struct VelocityConstraint {
    /// The contact normal.
    normal:             LV,

    /// The contact normal multiplied by the first body's inverse mass.
    weighted_normal1:   LV,
    /// The contact normal multiplied by the second body's inverse mass.
    weighted_normal2:   LV,

    /// The first body rotation axis.
    rot_axis1:          AV,
    /// The first body rotation axis multiplied by its inverse inertia.
    weighted_rot_axis1: AV,

    /// The second body rotation axis.
    rot_axis2:          AV,
    /// The second body rotation axis multiplied by its inverse inertia.
    weighted_rot_axis2: AV,

    /// The inverse of the sum of linear and angular inertia of both bodies.
    inv_projected_mass: N,

    /// The total impulse applied.
    impulse:            N,
    /// The lower bound of the impulse.
    lobound:            N,
    /// The upper bound of the impulse.
    hibound:            N,
    /// The target delta velocity.
    objective:          N,
    /// The id of the first body.
    id1:                int,
    /// The id of the second body.
    id2:                int,
    /// The id of the friction constraint.
    friction_limit_id:  uint,
    /// The friction coefficient on this contact.
    friction_coeff:     N
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
