//! Euler integration functions.

use na::{Translation, RotationWithTranslation};
use na;
use math::{Scalar, Point, Vect, Orientation, Matrix};

/// Explicit Euler integrator.
pub fn explicit_integrate(dt: Scalar, p: &Matrix, c: &Point, lv: &Vect, av: &Orientation, lf: &Vect, af: &Orientation) -> (Matrix, Vect, Orientation) {
    (
        displacement(dt.clone(), p, c, lv, av), 
        lv + lf * dt,
        av + af * dt
    )
}

/// Explicit Euler integrator. This will not update the rotational components.
pub fn explicit_integrate_wo_rotation(dt: Scalar, p: &Point, lv: &Vect, lf: &Vect) -> (Point, Vect) {
    (
        p  + lv * dt, 
        lv + lf * dt
    )
}

/// Semi-implicit Euler integrator.
pub fn semi_implicit_integrate(dt: Scalar, p: &Matrix, c: &Point, lv: &Vect, av: &Orientation, lf: &Vect, af: &Orientation) -> (Matrix, Vect, Orientation) {
    let nlv = lv + lf * dt;
    let nav = av + af * dt;

    (
        displacement(dt.clone(), p, c, &nlv, &nav),
        nlv,
        nav
    )
}

/// Semi-implicit Euler integrator. This will not update the rotational components.
pub fn semi_implicit_integrate_wo_rotation(dt: Scalar, p: &Point, lv: &Vect, lf: &Vect) -> (Point, Vect) {
    let nlv = lv + lf * dt;

    (
        p + nlv * dt,
        nlv
    )
}

/// Computes the transformation matrix required to move an object with a `lin_vel` linear velocity,
/// a `ang_vil` angular velocity, and a center of mass `center_of_mass`, during the time step `dt`.
pub fn displacement(dt: Scalar, _: &Matrix, center_of_mass: &Point, lin_vel: &Vect, ang_vel: &Orientation) -> Matrix {
    let mut res: Matrix = na::one();
    res.append_rotation_wrt_point(&(ang_vel * dt), center_of_mass.as_vec());

    res.append_translation(&(lin_vel * dt));

    res
}
