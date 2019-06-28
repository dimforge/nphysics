use na::{RealField, Unit};

use crate::force_generator::ForceGenerator;
use crate::math::{ForceType, Point, Vector};
use crate::object::{BodyPartHandle, BodyHandle, BodySlab, BodySet, Body};
use crate::solver::IntegrationParameters;

/// Generator of a force proportional to the distance separating two bodies.
pub struct Spring<N: RealField, Handle: BodyHandle> {
    b1: BodyPartHandle<Handle>,
    b2: BodyPartHandle<Handle>,
    anchor1: Point<N>,
    anchor2: Point<N>,
    length: N,
    stiffness: N,
}

impl<N: RealField, Handle: BodyHandle> Spring<N, Handle> {
    /// Initialize a spring attached to `b1` and `b2` at the points `anchor1` and `anchor2`.
    /// 
    /// Anchors are expressed in the local coordinates of the corresponding bodies.
    /// The spring has a rest length of `length` and a stiffness of `stiffness`.
    pub fn new(
        b1: BodyPartHandle<Handle>,
        b2: BodyPartHandle<Handle>,
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

impl<N: RealField, Handle: BodyHandle, Bodies: BodySet<N, Handle = Handle>> ForceGenerator<N, Bodies> for Spring<N, Handle> {
    fn apply(&mut self, _: &IntegrationParameters<N>, bodies: &mut Bodies) {
        let body1 = try_ret!(bodies.get(self.b1.0));
        let body2 = try_ret!(bodies.get(self.b2.0));
        let part1 = try_ret!(body1.part(self.b1.1));
        let part2 = try_ret!(body2.part(self.b2.1));

        let anchor1 = body1.world_point_at_material_point(part1, &self.anchor1);
        let anchor2 = body2.world_point_at_material_point(part2, &self.anchor2);

        let force_dir;
        let delta_length;

        if let Some((dir, length)) = Unit::try_new_and_get(anchor2 - anchor1, N::default_epsilon()) {
            force_dir = dir;
            delta_length = length - self.length;
        } else {
            force_dir = Vector::y_axis();
            delta_length = -self.length;
        }

        let force = force_dir.as_ref() * delta_length * self.stiffness;
        bodies.get_mut(self.b1.0).unwrap().apply_force_at_local_point(self.b1.1, &force, &self.anchor1, ForceType::Force, false);
        bodies.get_mut(self.b2.0).unwrap().apply_force_at_local_point(self.b2.1, &-force, &self.anchor2, ForceType::Force, false);
    }
}
