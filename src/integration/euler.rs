use std::num::One;
use nalgebra::traits::scalar_op::ScalarMul;
use nalgebra::traits::rotation;
use nalgebra::traits::rotation::Rotation;
use nalgebra::traits::translation::{Translation, Translatable};

pub fn explicit_integrate<M:  Translation<LV> + Translatable<LV, M2> + One,
                          M2: Rotation<AV> + Translation<LV>,
                          LV: Add<LV, LV> + ScalarMul<N> + Neg<LV>,
                          AV: Add<AV, AV> + ScalarMul<N>,
                          N:  Clone>(
                          dt: N,
                          p:  &M,
                          c:  &LV,
                          lv: &LV,
                          av: &AV,
                          lf: &LV,
                          af: &AV)
                          -> (M2, LV, AV) {
    (
        displacement(dt.clone(), p, c, lv, av), 
        integrate(dt.clone(), lv, lf),
        integrate(dt, av, af)
    )
}

pub fn explicit_integrate_wo_rotation<V: Add<V, V> + ScalarMul<N>,
                                      N: Clone>(
                                      dt: N,
                                      p:  &V,
                                      lv: &V,
                                      lf: &V)
                                      -> (V, V) {
    (
        integrate(dt.clone(), p, lv), 
        integrate(dt, lv, lf)
    )
}

pub fn semi_implicit_integrate<M:  Translation<LV> + Translatable<LV, M2> + One,
                               M2: Rotation<AV> + Translation<LV>,
                               LV: Add<LV, LV> + ScalarMul<N> + Neg<LV>,
                               AV: Add<AV, AV> + ScalarMul<N>,
                               N:  Clone>(
                               dt: N,
                               p:  &M,
                               c:  &LV,
                               lv: &LV,
                               av: &AV,
                               lf: &LV,
                               af: &AV)
                               -> (M2, LV, AV) {
    let nlv = integrate(dt.clone(), lv, lf);
    let nav = integrate(dt.clone(), av, af);

    (
        displacement(dt.clone(), p, c, &nlv, &nav),
        nlv,
        nav
    )
}

pub fn semi_implicit_integrate_wo_rotation<V: Add<V, V> + ScalarMul<N>,
                                           N: Clone>(
                                           dt: N,
                                           p:  &V,
                                           lv: &V,
                                           lf: &V)
                                           -> (V, V) {
    let nlv = integrate(dt.clone(), lv, lf);

    (
        integrate(dt.clone(), p, &nlv), 
        nlv
    )
}

// fn implicit_integrate<>()
// {
//    FIXME
// }

pub fn displacement<M: Translation<LV> + Translatable<LV, M2> + One,
                    M2: Rotation<AV> + Translation<LV>,
                    LV: ScalarMul<N> + Neg<LV>,
                    AV: ScalarMul<N>,
                    N>(
                    dt:             N,
                    _:              &M,
                    center_of_mass: &LV,
                    lin_vel:        &LV,
                    ang_vel:        &AV)
                    -> M2 {
    let mut res = rotation::rotated_wrt_point(&One::one::<M>(),
                                              &ang_vel.scalar_mul(&dt),
                                              center_of_mass);

    res.translate_by(&lin_vel.scalar_mul(&dt));

    res
}

fn integrate<N,
             V: Add<V, V> + ScalarMul<N>>(
             dt: N,
             v:  &V,
             f:  &V)
             -> V {
    v + f.scalar_mul(&dt)
}
