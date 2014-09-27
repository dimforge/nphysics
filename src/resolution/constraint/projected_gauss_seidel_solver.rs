use nalgebra::na;
use ncollide::math::{Vect, Orientation};
use resolution::constraint::velocity_constraint::VelocityConstraint;

/// Structure holding the result of the projected gauss seidel solver.
#[deriving(PartialEq, Show, Clone)]
pub struct Velocities {
    /// Linear velocity.
    pub lv: Vect,
    /// Angular velocity.
    pub av: Orientation
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
    // mj_lambda is result
    assert!(result.len() == num_bodies);

    for v in result.iter_mut() {
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
    for _ in range(0, num_iterations) {
        for c in restitution.iter_mut() {
            solve_velocity_constraint(c, result);
        }

        for c in friction.iter_mut() {
            let impulse = restitution[c.friction_limit_id].impulse.clone();

            if impulse > na::zero() {
                let bound = c.friction_coeff * impulse;
                c.lobound = -bound;
                c.hibound = bound;

                solve_velocity_constraint(c, result);
            }
        }
    }
}

#[inline(always)]
fn setup_warmstart_for_constraint(c: &VelocityConstraint, mj_lambda: &mut [Velocities]) {
    let id1 = c.id1;
    let id2 = c.id2;

    if id1 >= 0 {
        mj_lambda[id1 as uint].lv = mj_lambda[id1 as uint].lv - c.weighted_normal1 * c.impulse;
        mj_lambda[id1 as uint].av = mj_lambda[id1 as uint].av + c.weighted_rot_axis1 * c.impulse;
    }

    if id2 >= 0 {
        mj_lambda[id2 as uint].lv = mj_lambda[id2 as uint].lv + c.weighted_normal2 * c.impulse;
        mj_lambda[id2 as uint].av = mj_lambda[id2 as uint].av + c.weighted_rot_axis2 * c.impulse;
    }
}

#[inline(always)]
fn solve_velocity_constraint(c: &mut VelocityConstraint, mj_lambda: &mut [Velocities]) {
    let id1 = c.id1;
    let id2 = c.id2;

    let mut d_lambda_i = c.objective.clone();

    if id1 >= 0 {
        d_lambda_i = d_lambda_i + na::dot(&c.normal, &mj_lambda[id1 as uint].lv)
                                - na::dot(&c.rot_axis1, &mj_lambda[id1 as uint].av);
    }

    if id2 >= 0 {
        d_lambda_i = d_lambda_i - na::dot(&c.normal, &mj_lambda[id2 as uint].lv)
                                - na::dot(&c.rot_axis2, &mj_lambda[id2 as uint].av);
    }

    d_lambda_i = d_lambda_i * c.inv_projected_mass;

    // clamp the value such that: lambda- <= lambda <= lambda+
    // (this is the ``projected'' flavour of Gauss-Seidel
    let lambda_i_0 = c.impulse.clone();

    c.impulse = na::clamp(lambda_i_0 + d_lambda_i, c.lobound, c.hibound);

    d_lambda_i = c.impulse - lambda_i_0;


    if id1 >= 0 {
        mj_lambda[id1 as uint].lv = mj_lambda[id1 as uint].lv - c.weighted_normal1 * d_lambda_i;
        mj_lambda[id1 as uint].av = mj_lambda[id1 as uint].av + c.weighted_rot_axis1 * d_lambda_i;
    }

    if id2 >= 0 {
        mj_lambda[id2 as uint].lv = mj_lambda[id2 as uint].lv + c.weighted_normal2 * d_lambda_i;
        mj_lambda[id2 as uint].av = mj_lambda[id2 as uint].av + c.weighted_rot_axis2 * d_lambda_i;
    }
}
