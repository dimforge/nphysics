use std::num::One;
use nalgebra::traits::workarounds::scalar_op::ScalarMul;
use nalgebra::traits::rotation;
use nalgebra::traits::rotation::Rotation;
use nalgebra::traits::translation::Translation;
use body::dynamic::Dynamic;
use body::transformable::Transformable;

// FIXME: find a way to do the sampling: v *= 0.9998

#[inline(always)]
fn integrate_force<T, V: Add<V, V> + ScalarMul<T>>(dt: T, f: &V, v: &V) -> V
{ v + f.scalar_mul(&dt) }

fn displacement<T,
                M:  Translation<LV> + Rotation<AV> + One,
                LV: ScalarMul<T> + Neg<LV>,
                AV: ScalarMul<T>>
  (dt: T, orig: M, lin_vel: &LV, ang_vel: &AV) -> M
{
  rotation::rotate_wrt_point(&One::one::<M>(),
                             &ang_vel.scalar_mul(&dt),
                             &orig.translation())
    .translated(&lin_vel.scalar_mul(&dt))
}

pub fn integrate_body_velocity<RB: Dynamic<T, LV, AV, II>,
                               T:  Copy,
                               LV: Add<LV, LV> + ScalarMul<T>,
                               AV: Add<AV, AV> + ScalarMul<T>,
                               II>
       (dt: T, body: &mut RB)
{
  let lv = integrate_force(dt, &body.ext_lin_force(), &body.lin_vel());
  body.set_lin_vel(&lv);

  let av = integrate_force(dt, &body.ext_ang_force(), &body.ang_vel());
  body.set_ang_vel(&av);
}

pub fn integrate_body_position<RB: Dynamic<T, LV, AV, II> + Transformable<M>,
                               M:  Translation<LV> + Rotation<AV> + One,
                               LV: ScalarMul<T> + Neg<LV>,
                               AV: ScalarMul<T>,
                               T, II>
       (dt: T, body: &mut RB)
{
  let mv = displacement(dt,
                        body.local_to_world(),
                        &body.lin_vel(),
                        &body.ang_vel());

  body.append(&mv);
}
