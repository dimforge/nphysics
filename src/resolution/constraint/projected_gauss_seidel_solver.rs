use std::num::{Zero, Orderable};
use nalgebra::na::Vec;
use nalgebra::na;
use resolution::constraint::velocity_constraint::VelocityConstraint;

#[deriving(Eq, ToStr, Clone)]
pub struct Velocities<LV, AV> {
    lv: LV,
    av: AV
}

impl<LV: Zero, AV: Zero> Velocities<LV, AV> {
    pub fn new() -> Velocities<LV, AV> {
        Velocities {
            lv: na::zero(),
            av: na::zero()
        }
    }

    pub fn reset(&mut self) {
        self.lv = na::zero();
        self.av = na::zero();
    }
}

pub fn projected_gauss_seidel_solve<LV: Vec<N> + Clone,
                                    AV: Vec<N> + Clone,
                                    N:  Num + Orderable + Clone>(
                                    restitution:    &mut [VelocityConstraint<LV, AV, N>],
                                    friction:       &mut [VelocityConstraint<LV, AV, N>],
                                    result:         &mut [Velocities<LV, AV>],
                                    num_bodies:     uint,
                                    num_iterations: uint,
                                    is_lambda_zero: bool) {
    // initialize the solution with zeros...
    // MJLambda is result
    assert!(result.len() == num_bodies);

    for v in result.mut_iter() {
        v.reset();
    }

    // ... and warm start if possible
    if !is_lambda_zero {
        for c in restitution.iter() {
            setup_warmstart_for_constraint(c, result);
        }

        for c in friction.iter() {
            setup_warmstart_for_constraint(c, result);
        }
    }

    /*
     * solve the system
     */
    num_iterations.times(|| {
        for c in restitution.mut_iter() {
            solve_velocity_constraint(c, result);
        }

        for c in friction.mut_iter() {
            let impulse = restitution[c.friction_limit_id].impulse.clone();

            if impulse > na::zero() {
                let bound = c.friction_coeff * impulse;
                c.lobound = -bound;
                c.hibound = bound;

                solve_velocity_constraint(c, result);
            }
        }
    });
}

#[inline(always)]
pub fn setup_warmstart_for_constraint<LV: Vec<N> + Clone,
                                      AV: Vec<N> + Clone,
                                      N:  Num + Orderable + Clone>(
                                      c:        &VelocityConstraint<LV, AV, N>,
                                      MJLambda: &mut [Velocities<LV, AV>]) {
    let id1 = c.id1;
    let id2 = c.id2;

    if id1 >= 0 {
        MJLambda[id1 as uint].lv = MJLambda[id1 as uint].lv - c.weighted_normal1 * c.impulse;
        MJLambda[id1 as uint].av = MJLambda[id1 as uint].av + c.weighted_rot_axis1 * c.impulse;
    }

    if id2 >= 0 {
        MJLambda[id2 as uint].lv = MJLambda[id2 as uint].lv + c.weighted_normal2 * c.impulse;
        MJLambda[id2 as uint].av = MJLambda[id2 as uint].av + c.weighted_rot_axis2 * c.impulse;
    }
}

#[inline(always)]
pub fn solve_velocity_constraint<LV: Vec<N> + Clone,
                                 AV: Vec<N> + Clone,
                                 N:  Num + Orderable + Clone>(
                                 c:        &mut VelocityConstraint<LV, AV, N>,
                                 MJLambda: &mut [Velocities<LV, AV>]) {
    let id1 = c.id1;
    let id2 = c.id2;

    let mut d_lambda_i = c.objective.clone();

    if id1 >= 0 {
        d_lambda_i = d_lambda_i + na::dot(&c.normal, &MJLambda[id1 as uint].lv)
                                - na::dot(&c.rot_axis1, &MJLambda[id1 as uint].av);
    }

    if id2 >= 0 {
        d_lambda_i = d_lambda_i - na::dot(&c.normal, &MJLambda[id2 as uint].lv)
                                - na::dot(&c.rot_axis2, &MJLambda[id2 as uint].av);
    }

    d_lambda_i = d_lambda_i * c.inv_projected_mass;

    // clamp the value such that: lambda- <= lambda <= lambda+
    // (this is the ``projected'' flavour of Gauss-Seidel
    let lambda_i_0 = c.impulse.clone();

    c.impulse = (lambda_i_0 + d_lambda_i).clamp(&c.lobound, &c.hibound);

    d_lambda_i = c.impulse - lambda_i_0;


    if id1 >= 0 {
        MJLambda[id1 as uint].lv = MJLambda[id1 as uint].lv - c.weighted_normal1 * d_lambda_i;
        MJLambda[id1 as uint].av = MJLambda[id1 as uint].av + c.weighted_rot_axis1 * d_lambda_i;
    }

    if id2 >= 0 {
        MJLambda[id2 as uint].lv = MJLambda[id2 as uint].lv + c.weighted_normal2 * d_lambda_i;
        MJLambda[id2 as uint].av = MJLambda[id2 as uint].av + c.weighted_rot_axis2 * d_lambda_i;
    }
}

#[cfg(test)]
mod test {
    use super::projected_gauss_seidel_solve;
    use extra::test::BenchHarness;
    use nalgebra::vec::{Vec3, AlgebraicVec};
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

            constraint.hibound = Bounded::max_value();
            constraint.lobound = 0.0;

            constraint.objective = 2.0 * (i as f64);

            constraint.id1 = i;
            constraint.id2 = 1000 - 1 - i;

            constraints.push(constraint);
        }

        let mut empty = ~[];

        do bh.iter {
            projected_gauss_seidel_solve(constraints, empty, 1000, 40, false);
        }
    }
}
