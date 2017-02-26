use alga::general::Real;
use na;
use math::{Vector, Orientation};

#[derive(PartialEq, Debug, Clone)]
/// A constraint of velocity at a point of contact.
pub struct VelocityConstraint<N: Real> {
    /// The contact normal.
    pub normal:             Vector<N>,

    /// The contact normal multiplied by the first body's inverse mass.
    pub weighted_normal1:   Vector<N>,
    /// The contact normal multiplied by the second body's inverse mass.
    pub weighted_normal2:   Vector<N>,

    /// The first body rotation axis.
    pub rot_axis1:          Orientation<N>,
    /// The first body rotation axis multiplied by its inverse inertia.
    pub weighted_rot_axis1: Orientation<N>,

    /// The second body rotation axis.
    pub rot_axis2:          Orientation<N>,
    /// The second body rotation axis multiplied by its inverse inertia.
    pub weighted_rot_axis2: Orientation<N>,

    /// The inverse of the sum of linear and angular inertia of both bodies.
    pub inv_projected_mass: N,

    /// The total impulse applied.
    pub impulse:            N,
    /// The lower bound of the impulse.
    pub lobound:            N,
    /// The upper bound of the impulse.
    pub hibound:            N,
    /// The target delta velocity.
    pub objective:          N,
    /// The id of the first body.
    pub id1:                isize,
    /// The id of the second body.
    pub id2:                isize,
    /// The id of the friction constraint.
    pub friction_limit_id:  usize,
    /// The friction coefficient on this contact.
    pub friction_coeff:     N
}

impl<N: Real> VelocityConstraint<N> {
    /// Creates a new velocity constraint with all terms initialized to zero.
    pub fn new() -> VelocityConstraint<N> {
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
