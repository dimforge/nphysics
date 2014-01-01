use nalgebra::na::{Translation, RotationWithTranslation};
use nalgebra::na;
use ncollide::math::{N, V, LV, AV, M};

pub fn explicit_integrate(dt: N, p: &M, c: &LV, lv: &LV, av: &AV, lf: &LV, af: &AV) -> (M, LV, AV) {
    (
        displacement(dt.clone(), p, c, lv, av), 
        lv + lf * dt,
        av + af * dt
    )
}

pub fn explicit_integrate_wo_rotation(dt: N, p: &V, lv: &V, lf: &V) -> (V, V) {
    (
        p  + lv * dt, 
        lv + lf * dt
    )
}

pub fn semi_implicit_integrate(dt: N, p: &M, c: &LV, lv: &LV, av: &AV, lf: &LV, af: &AV) -> (M, LV, AV) {
    let nlv = lv + lf * dt;
    let nav = av + af * dt;

    (
        displacement(dt.clone(), p, c, &nlv, &nav),
        nlv,
        nav
    )
}

pub fn semi_implicit_integrate_wo_rotation(dt: N, p: &V, lv: &V, lf: &V) -> (V, V) {
    let nlv = lv + lf * dt;

    (
        p + nlv * dt,
        nlv
    )
}

// fn implicit_integrate<>()
// {
//    FIXME
// }

pub fn displacement(dt: N, _: &M, center_of_mass: &LV, lin_vel: &LV, ang_vel: &AV) -> M {
    let mut res: M = na::one();
    res.append_rotation_wrt_point(&(ang_vel * dt), center_of_mass);

    res.append_translation(&(lin_vel * dt));

    res
}
