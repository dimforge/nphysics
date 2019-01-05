use na::{Real, Unit};

use crate::force_generator::ForceGenerator;
use crate::math::{Force, Point, Vector};
use crate::object::{BodyPartHandle, BodySet};
use crate::solver::IntegrationParameters;

/// Generator of a force proportional to the distance separating two bodies.
pub struct Spring<N: Real> {
    b1: BodyPartHandle,
    b2: BodyPartHandle,
    anchor1: Point<N>,
    anchor2: Point<N>,
    length: N,
    stiffness: N,
}

impl<N: Real> Spring<N> {
    /// Initialize a spring attached to `b1` and `b2` at the points `anchor1` and `anchor2`.
    /// 
    /// Anchors are expressed in the local coordinates of the corresponding bodies.
    /// The spring has a rest length of `length` and a stiffness of `stiffness`.
    pub fn new(
        b1: BodyPartHandle,
        b2: BodyPartHandle,
        anchor1: Point<N>,
        anchor2: Point<N>,
        length: N,
        stiffness: N,
    ) -> Self {
        Spring {
            b1,
            b2,
            anchor1,
            anchor2,
            length,
            stiffness,
        }
    }

    /// Sets the attach point to the first body.
    /// 
    /// The anchor is expressed in the local coordinatse of the first body.
    pub fn set_anchor_1(&mut self, anchor: Point<N>) {
        self.anchor1 = anchor;
    }

    /// Sets the attach point to the second body.
    /// 
    /// The anchor is expressed in the local coordinatse of the second body.
    pub fn set_anchor_2(&mut self, anchor: Point<N>) {
        self.anchor2 = anchor
    }
}

impl<N: Real> ForceGenerator<N> for Spring<N> {
    fn apply(&mut self, _: &IntegrationParameters<N>, bodies: &mut BodySet<N>) -> bool {
        if !bodies.contains_body(self.b1.0) || !bodies.contains_body(self.b2.0) {
            return false;
        }

        let part1 = bodies.body(self.b1.0).and_then(|b| b.part(self.b1.1));
        let part2 = bodies.body(self.b2.0).and_then(|b| b.part(self.b2.1));

        if let (Some(part1), Some(part2)) = (part1, part2) {
            let anchor1 = part1.position() * self.anchor1;
            let anchor2 = part2.position() * self.anchor2;

            let force_dir;
            let delta_length;

            if let Some((dir, length)) = Unit::try_new_and_get(anchor2 - anchor1, N::default_epsilon())
                {
                    force_dir = dir;
                    delta_length = length - self.length;
                } else {
                force_dir = Vector::y_axis();
                delta_length = -self.length;
            }

            let force = Force::linear(force_dir.as_ref() * delta_length * self.stiffness);
            bodies.body_mut(self.b1.0).unwrap().part_mut(self.b1.1).unwrap().apply_force(&force);
            bodies.body_mut(self.b2.0).unwrap().part_mut(self.b2.1).unwrap().apply_force(&-force);

            true
        } else {
            false
        }
    }
}
