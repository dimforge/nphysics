use std::num::{Zero, Orderable};
use std::vec;
use nalgebra::traits::dot::Dot;
use nalgebra::traits::vector_space::VectorSpace;
use nalgebra::traits::division_ring::DivisionRing;
use resolution::constraint::velocity_constraint::VelocityConstraint;

#[deriving(Eq, ToStr, Clone)]
pub struct Velocities<LV, AV> {
    lv: LV,
    av: AV
}

impl<LV: Zero, AV: Zero> Velocities<LV, AV> {
    fn new() -> Velocities<LV, AV> {
        Velocities {
            lv: Zero::zero(),
            av: Zero::zero()
        }
    }
}

pub fn projected_gauss_seidel_solve<LV: VectorSpace<N> + Dot<N> + Clone + ToStr,
                                    AV: VectorSpace<N> + Dot<N> + Clone + ToStr,
                                    N:  DivisionRing + Orderable + Clone + ToStr>(
                                    constraints:    &mut [VelocityConstraint<LV, AV, N>],
                                    num_bodies:     uint,
                                    num_iterations: uint,
                                    is_lambda_zero: bool)
                                    -> ~[Velocities<LV, AV>] {
    unsafe {
        // initialize the solution with zeros...
        // FIXME: avoid allocation at each update?
        let mut MJLambda = vec::from_elem(num_bodies, Velocities::new::<LV, AV>());

        // ... and warm start if possible
        if !is_lambda_zero {
            for c in constraints.iter() {
                let id1 = c.id1;
                let id2 = c.id2;

                if id1 >= 0 {
                    let mjl1 = MJLambda.unsafe_mut_ref(id1 as uint);
                    (*mjl1).lv = (*mjl1).lv - c.weighted_normal1.scalar_mul(&c.impulse);
                    (*mjl1).av = (*mjl1).av + c.weighted_rot_axis1.scalar_mul(&c.impulse);
                }

                if id2 >= 0 {
                    let mjl2 = MJLambda.unsafe_mut_ref(id1 as uint);
                    (*mjl2).lv = (*mjl2).lv + c.weighted_normal2.scalar_mul(&c.impulse);
                    (*mjl2).av = (*mjl2).av + c.weighted_rot_axis2.scalar_mul(&c.impulse);
                }
            }
        }

        /*
         * solve the system
         */
        do num_iterations.times {
            for c in constraints.mut_iter() {
                let id1 = c.id1;
                let id2 = c.id2;

                let mjl1 = MJLambda.unsafe_mut_ref(id1 as uint);
                let mjl2 = MJLambda.unsafe_mut_ref(id2 as uint);

                let mut d_lambda_i = c.objective.clone();

                if id1 >= 0 {
                    d_lambda_i = d_lambda_i + c.normal.dot(&(*mjl1).lv) - c.rot_axis1.dot(&(*mjl1).av);
                }

                if id2 >= 0 {
                    d_lambda_i = d_lambda_i - c.normal.dot(&(*mjl2).lv) - c.rot_axis2.dot(&(*mjl2).av);
                }

                d_lambda_i = d_lambda_i * c.inv_projected_mass;

                // clamp the value such that: lambda- <= lambda <= lambda+
                // (this is the ``projected'' flavour of Gauss-Seidel
                let lambda_i_0 = c.impulse.clone();

                c.impulse = (lambda_i_0 + d_lambda_i).clamp(&c.lobound, &c.hibound);

                d_lambda_i = c.impulse - lambda_i_0;


                if id1 >= 0 {
                    (*mjl1).lv = (*mjl1).lv - c.weighted_normal1.scalar_mul(&d_lambda_i);
                    (*mjl1).av = (*mjl1).av + c.weighted_rot_axis1.scalar_mul(&d_lambda_i);
                }

                if id2 >= 0 {
                    (*mjl2).lv = (*mjl2).lv + c.weighted_normal2.scalar_mul(&d_lambda_i);
                    (*mjl2).av = (*mjl2).av + c.weighted_rot_axis2.scalar_mul(&d_lambda_i);
                }
            }
        }

        MJLambda
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use extra::test::BenchHarness;
    use nalgebra::vec::Vec3;
    use nalgebra::traits::norm::Norm;
    use resolution::constraint::velocity_constraint::VelocityConstraint;

    #[bench]
    fn bench_pgs(bh: &mut BenchHarness) {
        let mut constraints = ~[];

        // initialize the constraints
        for i in range(0, 1000) {
            let mut constraint = VelocityConstraint::new();

            constraint.normal = Vec3::new(i as f64 * 1.0, 3.0f64, i as f64).normalized();
            constraint.weighted_normal1 = Vec3::new(4.0, 3.0f64, i as f64);
            constraint.weighted_normal2 = Vec3::new(4.0, 3.0f64, i as f64);

            constraint.rot_axis1 = Vec3::new(i as f64 * 1.0, 3.0f64, i as f64).normalized();
            constraint.rot_axis2 = Vec3::new(-i as f64 * -1.0, 3.0f64, i as f64).normalized();
            constraint.weighted_rot_axis1 = Vec3::new(4.0, 3.0f64, i as f64);
            constraint.weighted_rot_axis2 = Vec3::new(4.0, 3.0f64, i as f64);

            constraint.inv_projected_mass = 42.0;

            constraint.hibound  = Bounded::max_value::<f64>();
            constraint.lobound = 0.0;

            constraint.objective = 2.0 * (i as f64);

            constraint.id1 = i;
            constraint.id2 = 1000 - 1 - i;

            constraints.push(constraint);
        }

        do bh.iter {
            projected_gauss_seidel_solve(constraints, 1000, 40, false);
        }
    }
}
