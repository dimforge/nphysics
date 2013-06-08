// use integrator::integrator::Integrator;

// FIXME: this does not compile… why?
// impl<I1: Integrator<T, RB>, I2: Integrator<T, RB>, T, RB>
//     Integrator<T, RB> for (I1, I2)
// {
//   fn integrate(&self, dt: T, b: &mut RB)
//   {
//     let &(a, b) = self; // FIXME: will this do a copy?
//     a.integrate(dt, &mut b);
//     b.integrate(dt, &mut b);
//   }
// }
