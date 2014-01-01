use nalgebra::na;
use ncollide::math::{N, LV, AV};

#[deriving(Eq, ToStr, Clone)]
pub struct VelocityConstraint {
    normal:             LV,

    weighted_normal1:   LV,
    weighted_normal2:   LV,

    rot_axis1:          AV,
    weighted_rot_axis1: AV,

    rot_axis2:          AV,
    weighted_rot_axis2: AV,

    inv_projected_mass: N,

    impulse:            N,
    lobound:            N,
    hibound:            N,
    objective:          N,
    id1:                int,
    id2:                int,
    friction_limit_id:  uint,
    friction_coeff:     N
}

impl VelocityConstraint {
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
