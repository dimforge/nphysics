use std::num::One;
use nalgebra::traits::scalar_op::ScalarMul;
use nalgebra::traits::rotation;
use nalgebra::traits::rotation::Rotation;
use nalgebra::traits::translation::{Translation, Translatable};
use nalgebra::traits::transformation::Transformation;
use body::dynamic::Dynamic;

// FIXME: find a way to do the sampling: v *= 0.9998

#[inline(always)]
fn integrate_force<N, V: Add<V, V> + ScalarMul<N>>(dt: N, f: &V, v: &V) -> V
{ v + f.scalar_mul(&dt) }

fn displacement<M: Translation<LV> + Translatable<LV, M2> + One,
                M2: Rotation<AV> + Translation<LV>,
                LV: ScalarMul<N> + Neg<LV>,
                AV: ScalarMul<N>,
                N>
  (dt: N, orig: M, lin_vel: &LV, ang_vel: &AV) -> M2
{
  let mut res = rotation::rotate_wrt_point(&One::one::<M>(),
                                           &ang_vel.scalar_mul(&dt),
                                           &orig.translation());

  res.translate(&lin_vel.scalar_mul(&dt));

  res
}

pub fn integrate_body_velocity<RB: Dynamic<N, LV, AV, II>,
                               N:  Copy,
                               LV: Add<LV, LV> + ScalarMul<N>,
                               AV: Add<AV, AV> + ScalarMul<N>,
                               II>
       (dt: N, body: &mut RB)
{
  let lv = integrate_force(copy dt, &body.ext_lin_force(), &body.lin_vel());
  body.set_lin_vel(&lv);

  let av = integrate_force(dt, &body.ext_ang_force(), &body.ang_vel());
  body.set_ang_vel(&av);
}

pub fn integrate_body_position<RB: Dynamic<N, LV, AV, II> + Transformation<M>,
                               M: Rotation<AV> + Translation<LV> +
                                  Translatable<LV, M> + One,
                               LV: ScalarMul<N> + Neg<LV>,
                               AV: ScalarMul<N>,
                               N,
                               II>
       (dt: N, body: &mut RB)
{
  let mv = displacement(dt,
                        body.transformation(),
                        &body.lin_vel(),
                        &body.ang_vel());

  body.transform_by(&mv);
}
