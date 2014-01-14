use std::num::Orderable;
use nalgebra::na;
use ncollide::math::{LV, AV};
use resolution::constraint::velocity_constraint::VelocityConstraint;

/// Structure holding the result of the projected gauss seidel solver.
#[deriving(Eq, ToStr, Clone)]
pub struct Velocities {
    /// Linear velocity.
    lv: LV,
    /// Angular velocity.
    av: AV
}

impl Velocities {
    /// Creates a new `Velocities`.
    pub fn new() -> Velocities {
        Velocities {
            lv: na::zero(),
            av: na::zero()
        }
    }

    /// Reset this structure to zero.
    pub fn reset(&mut self) {
        self.lv = na::zero();
        self.av = na::zero();
    }
}

/// Solve a set of velocity constraints using the projected gauss seidel solver.
///
/// # Arguments:
/// * `restitution` - constraints to simulate the restitution.
/// * `friction`    - constraints to simulate friction.
/// * `result`      - vector which will contain the result afterward. Must have the size
/// `num_bodies`.
/// * `num_bodies`  - the size of `result`.
/// * `num_iterations` - the number of iterations to perform.
/// * `is_lambda_zero` - indicates whether or not the every element of `result` has been
/// reinitialized. Set this to `false` if the `result` comes from a previous execution of
/// `projected_gauss_seidel_solve`: this will perform warm-starting.
pub fn projected_gauss_seidel_solve(restitution:    &mut [VelocityConstraint],
                                    friction:       &mut [VelocityConstraint],
                                    result:         &mut [Velocities],
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
fn setup_warmstart_for_constraint(c: &VelocityConstraint, MJLambda: &mut [Velocities]) {
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
fn solve_velocity_constraint(c: &mut VelocityConstraint, MJLambda: &mut [Velocities]) {
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

#[cfg(test, dim3, f32)]
mod test {
    use super::{Velocities, projected_gauss_seidel_solve};
    use extra::test::BenchHarness;
    use nalgebra::na::Vec3;
    use nalgebra::na;
    use resolution::constraint::velocity_constraint::VelocityConstraint;

    #[bench]
    fn bench_pgs(bh: &mut BenchHarness) {
        let mut constraints = ~[];
        let mut res   = ~[];

        // initialize the constraints
        for i in range(0, 1000) {
            let mut constraint = VelocityConstraint::new();

            constraint.normal = na::normalize(&Vec3::new(i as f32 * 1.0, 3.0f32, i as f32));
            constraint.weighted_normal1 = na::normalize(&Vec3::new(4.0, 3.0f32, i as f32));
            constraint.weighted_normal2 = na::normalize(&Vec3::new(4.0, 3.0f32, i as f32));

            constraint.rot_axis1 = na::normalize(&Vec3::new(i as f32 * 1.0, 3.0f32, i as f32));
            constraint.rot_axis2 = na::normalize(&Vec3::new(-i as f32 * -1.0, 3.0f32, i as f32));
            constraint.weighted_rot_axis1 = Vec3::new(4.0, 3.0f32, i as f32);
            constraint.weighted_rot_axis2 = Vec3::new(4.0, 3.0f32, i as f32);

            constraint.inv_projected_mass = 42.0;

            constraint.hibound = Bounded::max_value();
            constraint.lobound = 0.0;

            constraint.objective = 2.0 * (i as f32);

            constraint.id1 = i;
            constraint.id2 = 1000 - 1 - i;

            constraints.push(constraint);
            res.push(Velocities::new());
        }

        let mut empty = ~[];

        bh.iter(|| {
            projected_gauss_seidel_solve(constraints, empty, res, 1000, 40, false);
        });
    }
}
