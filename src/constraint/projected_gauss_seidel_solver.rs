use std::num::{Zero, Orderable};
use std::uint;
use std::vec;
use nalgebra::traits::division_ring::DivisionRing;

pub fn projected_gauss_seidel_solve<N: DivisionRing + Orderable + Copy>
       (J:              &mut [N],
        MJ:             &mut [N],
        b:              &mut [N],
        lambda:         &mut [N],
        bounds:         &mut [N],
        idx:            &mut [int],
        sparcity:       uint,
        num_equations:  uint,
        num_bodies:     uint,
        num_iterations: uint) -> ~[N]
{
  // JMJ matrix’s diagonal.
  let mut d        = vec::from_elem(num_equations, Zero::zero::<N>());
  // MJLambda = MJ * lambda
  let mut MJLambda = vec::from_elem(sparcity * num_bodies, Zero::zero::<N>());

  /*
   * compute MJLambda = MJ * lambda
   */
  // warm start
  for uint::iterate(0u, num_equations) |i|
  {
    let b1 = idx[i * 2];
    let b2 = idx[i * 2 + 1];

    if (b1 >= 0)
    {
      for uint::iterate(0u, sparcity) |j|
      { MJLambda[b1 as uint * sparcity + j] += MJ[i * sparcity * 2 + j] * lambda[i]; }
    }

    if (b2 >= 0)
    {
      for uint::iterate(0u, sparcity) |j|
      { MJLambda[b2 as uint * sparcity + j] += MJ[i * sparcity * 2 + sparcity + j] * lambda[i]; }
    }
  }

  /*
   * init JB's diagonal
   */
  for uint::iterate(0u, num_equations) |i|
  {
    /*
     * J and MJ^t have the same sparcity
     */
    d[i] = Zero::zero();

    for uint::iterate(0u, sparcity) |j|
    {
      d[i] +=
        J[i * sparcity * 2 + j] * MJ[i * sparcity * 2 + j]
        + J[i * sparcity * 2 + j + sparcity] * MJ[sparcity + i * sparcity * 2 + j];
    }
  }

  /*
   * solve the system
   */
  for num_iterations.times
  {
    for uint::iterate(0u, num_equations) |i|
    {
      let b1 = idx[i * 2 + 0] * (sparcity as int);
      let b2 = idx[i * 2 + 1] * (sparcity as int);

      let mut d_lambda_i = b[i];

      if (b1 >= 0)
      {
        for uint::iterate(0u, sparcity) |j|
        { d_lambda_i -= J[2 * sparcity * i + j] * MJLambda[b1 as uint + j] }
      }

      if (b2 >= 0)
      {
        for uint::iterate(0u, sparcity) |j|
        { d_lambda_i -= J[2 * sparcity * i + sparcity + j] * MJLambda[b2 as uint + j] }
      }

      d_lambda_i /= d[i];

      /*
       * clamp the value such that: lambda- <= lambda <= lambda+
       * (this is the ``projected'' flavour of Gauss-Seidel
       */
      let lambda_i_0 = lambda[i];

      // FIXME: clamp takes pointers as arguments… would it be faster if it did
      // not?
      lambda[i] = (lambda_i_0 + d_lambda_i).clamp(&bounds[i * 2], &bounds[i * 2 + 1]);

      d_lambda_i = lambda[i] - lambda_i_0;


      if (b1 >= 0)
      {
        for uint::iterate(0u, sparcity) |j|
        { MJLambda[b1 as uint + j] += d_lambda_i * MJ[i * sparcity * 2 + j] }
      }

      if (b2 >= 0)
      {
        for uint::iterate(0u, sparcity) |j|
        { MJLambda[b2 as uint + j] += d_lambda_i * MJ[i * sparcity * 2 + sparcity + j] }
      }
    }
  }

  MJLambda
}
