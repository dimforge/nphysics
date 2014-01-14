//! Euler integration functions.

use nalgebra::na::{Translation, RotationWithTranslation};
use nalgebra::na;
use ncollide::math::{N, V, LV, AV, M};

/// Explicit Euler integrator.
pub fn explicit_integrate(dt: N, p: &M, c: &LV, lv: &LV, av: &AV, lf: &LV, af: &AV) -> (M, LV, AV) {
    (
        displacement(dt.clone(), p, c, lv, av), 
        lv + lf * dt,
        av + af * dt
    )
}

/// Explicit Euler integrator. This will not update the rotational components.
pub fn explicit_integrate_wo_rotation(dt: N, p: &V, lv: &V, lf: &V) -> (V, V) {
    (
        p  + lv * dt, 
        lv + lf * dt
    )
}

/// Semi-implicit Euler integrator.
pub fn semi_implicit_integrate(dt: N, p: &M, c: &LV, lv: &LV, av: &AV, lf: &LV, af: &AV) -> (M, LV, AV) {
    let nlv = lv + lf * dt;
    let nav = av + af * dt;

    (
        displacement(dt.clone(), p, c, &nlv, &nav),
        nlv,
        nav
    )
}

/// Semi-implicit Euler integrator. This will not update the rotational components.
pub fn semi_implicit_integrate_wo_rotation(dt: N, p: &V, lv: &V, lf: &V) -> (V, V) {
    let nlv = lv + lf * dt;

    (
        p + nlv * dt,
        nlv
    )
}

/// Computes the transformation matrix required to move an object with a `lin_vel` linear velocity,
/// a `ang_vil` angular velocity, and a center of mass `center_of_mass`, during the time step `dt`.
pub fn displacement(dt: N, _: &M, center_of_mass: &LV, lin_vel: &LV, ang_vel: &AV) -> M {
    let mut res: M = na::one();
    res.append_rotation_wrt_point(&(ang_vel * dt), center_of_mass);

    res.append_translation(&(lin_vel * dt));

    res
}
