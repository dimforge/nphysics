use std::num::{Zero, Orderable};
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
        num_iterations: uint,
        is_lambda_zero: bool) -> ~[N]
{
  // JMJ matrix’s diagonal.
  let mut d        = vec::from_elem(num_equations, Zero::zero::<N>());
  // MJLambda = MJ * lambda
  let mut MJLambda = vec::from_elem(sparcity * num_bodies, Zero::zero::<N>());

  /*
   * compute MJLambda = MJ * lambda
   */
  // warm start
  if !is_lambda_zero
  {
    let mut i = 0;
    while(i != num_equations)
    {
      let b1 = unsafe { idx.unsafe_get(i * 2)     } * (sparcity as int);
      let b2 = unsafe { idx.unsafe_get(i * 2 + 1) } * (sparcity as int);

      if b1 >= 0
      {
        let mut j = 0;
        while(j != sparcity)
        {
          unsafe {
            *MJLambda.unsafe_mut_ref(b1 as uint + j) =
              *MJLambda.unsafe_mut_ref(b1 as uint + j) +
              MJ.unsafe_get(i * sparcity * 2 + j) * lambda.unsafe_get(i);
          }

          j = j + 1;
        }
      }

      if b2 >= 0
      {
        let mut j = 0;
        while(j != sparcity)
        {
          unsafe {
            *MJLambda.unsafe_mut_ref(b2 as uint + j) =
              *MJLambda.unsafe_mut_ref(b2 as uint + j) +
              MJ.unsafe_get(i * sparcity * 2 + sparcity + j) *
              lambda.unsafe_get(i);
          }

          j = j + 1;
        }
      }

      i = i + 1;
    }
  }

  /*
   * init JB's diagonal
   */
  let mut i = 0;
  while (i != num_equations)
  {
    /*
     * J and MJ^t have the same sparcity
     */
    unsafe {
      d.unsafe_set(i, Zero::zero());
    }

    let mut j = 0;
    while(j != sparcity)
    {
      unsafe {
        *d.unsafe_mut_ref(i) =
          *d.unsafe_mut_ref(i) +
          J.unsafe_get(i * sparcity * 2 + j) *
          MJ.unsafe_get(i * sparcity * 2 + j)
          + J.unsafe_get(i * sparcity * 2 + j + sparcity) *
            MJ.unsafe_get(sparcity + i * sparcity * 2 + j);
      }

      j = j + 1;
    }

    i = i + 1;
  }

  /*
   * solve the system
   */

  let mut time = 0;
  while(time != num_iterations)
  {
    let mut i = 0;
    while(i != num_equations)
    {
      let b1 = unsafe { idx.unsafe_get(i * 2 + 0) } * (sparcity as int);
      let b2 = unsafe { idx.unsafe_get(i * 2 + 1) } * (sparcity as int);

      let mut d_lambda_i = unsafe { b.unsafe_get(i) };

      if b1 >= 0
      {
        let mut j = 0;
        while(j != sparcity)
        {
          unsafe {
            d_lambda_i = d_lambda_i - J.unsafe_get(2 * sparcity * i + j) *
                          MJLambda.unsafe_get(b1 as uint + j)
          }

          j = j + 1;
        }
      }

      if b2 >= 0
      {
        let mut j = 0;
        while(j != sparcity)
        {
          unsafe {
            d_lambda_i = d_lambda_i - J.unsafe_get(2 * sparcity * i + sparcity + j) *
                         MJLambda.unsafe_get(b2 as uint + j)
          }

          j = j + 1;
        }
      }

      d_lambda_i = d_lambda_i / unsafe { d.unsafe_get(i) };

      /*
       * clamp the value such that: lambda- <= lambda <= lambda+
       * (this is the ``projected'' flavour of Gauss-Seidel
       */
      let lambda_i_0 = unsafe { lambda.unsafe_get(i) };

      // FIXME: clamp takes pointers as arguments… would it be faster if it did
      // not?
      unsafe {
        lambda.unsafe_set(
          i,
          (lambda_i_0 + d_lambda_i).clamp(&bounds.unsafe_get(i * 2),
                                          &bounds.unsafe_get(i * 2 + 1))
        );
      }

      d_lambda_i = unsafe { lambda.unsafe_get(i) } - lambda_i_0;


      if b1 >= 0
      {
        let mut j = 0;
        while(j != sparcity)
        {
          unsafe {
            *MJLambda.unsafe_mut_ref(b1 as uint + j) =
              *MJLambda.unsafe_mut_ref(b1 as uint + j) +
              d_lambda_i * MJ.unsafe_get(i * sparcity * 2 + j)
          }

          j = j + 1;
        }
      }

      if b2 >= 0
      {
        let mut j = 0;
        while(j != sparcity)
        {
          unsafe {
            *MJLambda.unsafe_mut_ref(b2 as uint + j) =
              *MJLambda.unsafe_mut_ref(b2 as uint + j) +
              d_lambda_i * MJ.unsafe_get(i * sparcity * 2 + sparcity + j)
          }

          j = j + 1;
        }
      }

      i = i + 1;
    }

    time = time + 1;
  }

  MJLambda
}
