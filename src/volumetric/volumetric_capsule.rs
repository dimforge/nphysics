use crate::volumetric::cylinder_volume;
use crate::volumetric::ball_volume;

/// Computes the volume of a capsule.
pub fn capsule_volume(half_height: &N, radius: &N) -> N {
    cylinder_volume(half_height, radius) + ball_volume(radius)
}
