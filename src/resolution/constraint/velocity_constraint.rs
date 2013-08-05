use std::num::Zero;

#[deriving(Eq, ToStr, Clone)]
pub struct VelocityConstraint<LV, AV, N>
{
    normal:             LV,

    weighted_normal1:   LV,
    weighted_normal2:   LV,

    rot_axis1:          AV,
    weighted_rot_axis1: AV,

    rot_axis2:          AV,
    weighted_rot_axis2: AV,

    projected_mass:     N,

    impulse:            N,
    unit_impulse:       N,
    lobound:            N,
    hibound:            N,
    objective:          N,
    id1:                int,
    id2:                int
}

impl<LV: Zero, AV: Zero, N: Zero> VelocityConstraint<LV, AV, N>
{
    pub fn new() -> VelocityConstraint<LV, AV, N>
    {
        VelocityConstraint {
            normal:             Zero::zero(),

            weighted_normal1:   Zero::zero(),
            weighted_normal2:   Zero::zero(),

            rot_axis1:          Zero::zero(),
            weighted_rot_axis1: Zero::zero(),

            rot_axis2:          Zero::zero(),
            weighted_rot_axis2: Zero::zero(),

            projected_mass:     Zero::zero(),

            impulse:            Zero::zero(),
            unit_impulse:       Zero::zero(),
            hibound:            Zero::zero(),
            lobound:            Zero::zero(),
            objective:          Zero::zero(),
            id1:                -1,
            id2:                -1
        }
    }
}
