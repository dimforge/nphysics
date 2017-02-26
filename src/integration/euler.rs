//! Euler integration functions.

use na;
use alga::general::Real;
use math::{Point, Vector, Orientation, Rotation, Translation, Isometry};

/// Explicit Euler integrator.
pub fn explicit_integrate<N: Real>(dt: N,
                                   p:  &Isometry<N>, c:  &Point<N>,
                                   lv: &Vector<N>, av: &Orientation<N>,
                                   lf: &Vector<N>, af: &Orientation<N>)
                                   -> (Isometry<N>, Vector<N>, Orientation<N>) {
    (
        displacement(dt.clone(), p, c, lv, av), 
        *lv + *lf * dt,
        *av + *af * dt
    )
}

/// Explicit Euler integrator. This will not update the rotational components.
pub fn explicit_integrate_wo_rotation<N: Real>(dt: N, p: &Point<N>, lv: &Vector<N>, lf: &Vector<N>)
                                               -> (Point<N>, Vector<N>) {
    (
        *p  + *lv * dt, 
        *lv + *lf * dt
    )
}

/// Semi-implicit Euler integrator.
pub fn semi_implicit_integrate<N: Real>(dt: N,
                                        p:     &Isometry<N>, c:     &Point<N>,
                                        lv:    &Vector<N>, av:    &Orientation<N>,
                                        l_acc: &Vector<N>, a_acc: &Orientation<N>)
                                        -> (Isometry<N>, Vector<N>, Orientation<N>) {
    let nlv = *lv + *l_acc * dt;
    let nav = *av + *a_acc * dt;

    (
        displacement(dt.clone(), p, c, &nlv, &nav),
        nlv,
        nav
    )
}

/// Semi-implicit Euler integrator. This will not update the rotational components.
pub fn semi_implicit_integrate_wo_rotation<N: Real>(dt: N, p: &Point<N>, lv: &Vector<N>, lf: &Vector<N>)
                                                    -> (Point<N>, Vector<N>) {
    let nlv = *lv + *lf * dt;

    (
        *p + nlv * dt,
        nlv
    )
}

/// Computes the transformation matrix required to move an object with a `lin_vel` linear velocity,
/// a `ang_vil` angular velocity, and a center of mass `center_of_mass`, during the time step `dt`.
pub fn displacement<N: Real>(dt: N, _: &Isometry<N>, center_of_mass: &Point<N>,
                             lin_vel: &Vector<N>, ang_vel: &Orientation<N>) -> Isometry<N> {
    let mut res: Isometry<N> = na::one();
    let rot = Rotation::from_scaled_axis(ang_vel * dt);
    res.append_rotation_wrt_point_mut(&rot, center_of_mass);
    res.append_translation_mut(&Translation::from_vector(lin_vel * dt));

    res
}
