use std::num::{Zero, Orderable};
use std::vec;
use nalgebra::traits::dot::Dot;
use nalgebra::traits::vector_space::VectorSpace;
use nalgebra::traits::division_ring::DivisionRing;
use resolution::constraint::velocity_constraint::VelocityConstraint;

#[deriving(Eq, ToStr, Clone)]
pub struct Velocities<LV, AV>
{
    lv: LV,
    av: AV
}

impl<LV: Zero, AV: Zero> Velocities<LV, AV>
{
    fn new() -> Velocities<LV, AV>
    {
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
                                    -> ~[Velocities<LV, AV>]
{
    // initialize the solution with zeros...
    // FIXME: avoid allocation at each update?
    let mut MJLambda = vec::from_elem(num_bodies, Velocities::new::<LV, AV>());

    // ... and warm start if possible
    if !is_lambda_zero
    {
        for c in constraints.iter()
        {
            let id1 = c.id1;
            let id2 = c.id2;

            if id1 >= 0
            {
                MJLambda[id1].lv = MJLambda[id1].lv - c.weighted_normal1.scalar_mul(&c.impulse);
                MJLambda[id1].av = MJLambda[id1].av + c.weighted_rot_axis1.scalar_mul(&c.impulse);
            }

            if id2 >= 0
            {
                MJLambda[id2].lv = MJLambda[id2].lv + c.weighted_normal2.scalar_mul(&c.impulse);
                MJLambda[id2].av = MJLambda[id2].av + c.weighted_rot_axis2.scalar_mul(&c.impulse);
            }
        }
    }

    /*
     * solve the system
     */
    do num_iterations.times
    {
        for c in constraints.mut_iter()
        {
            let id1 = c.id1;
            let id2 = c.id2;

            let mut d_lambda_i = c.objective.clone();

            if id1 >= 0
            {
                d_lambda_i = d_lambda_i + c.normal.dot(&MJLambda[id1].lv)
                    - c.rot_axis1.dot(&MJLambda[id1].av);
            }

            if id2 >= 0
            {
                d_lambda_i = d_lambda_i - c.normal.dot(&MJLambda[id2].lv)
                    - c.rot_axis2.dot(&MJLambda[id2].av);
            }

            d_lambda_i = d_lambda_i / c.projected_mass;

            // clamp the value such that: lambda- <= lambda <= lambda+
            // (this is the ``projected'' flavour of Gauss-Seidel
            let lambda_i_0 = c.impulse.clone();

            c.impulse = (lambda_i_0 + d_lambda_i).clamp(&c.lobound, &c.hibound);

            d_lambda_i = c.impulse - lambda_i_0;


            if id1 >= 0
            {
                MJLambda[id1].lv = MJLambda[id1].lv - c.weighted_normal1.scalar_mul(&d_lambda_i);
                MJLambda[id1].av = MJLambda[id1].av + c.weighted_rot_axis1.scalar_mul(&d_lambda_i);
            }

            if id2 >= 0
            {
                MJLambda[id2].lv = MJLambda[id2].lv + c.weighted_normal2.scalar_mul(&d_lambda_i);
                MJLambda[id2].av = MJLambda[id2].av + c.weighted_rot_axis2.scalar_mul(&d_lambda_i);
            }
        }
    }

    MJLambda
}
