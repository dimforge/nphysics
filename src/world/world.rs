use std::vec;
use integrator::integrator::Integrator;

pub struct World<RB, I, NF, BP, CD, CS, T>
{
  priv bodies      : ~[@mut RB],
  priv integrator  : ~I,
  // priv coll_graph  : @Graph<RB, NF>,
  priv broad_phase : ~BP,
  priv narrow_phase: ~NF,
  priv solver      : ~CS
}

impl<RB,
     I : Integrator<T, RB>,
     NF,
     BP,
     CD,
     CS,
     T: Copy>
World<RB, I, NF, BP, CD, CS, T>
{
  pub fn new(integrator:   ~I,
             broad_phase:  ~BP,
             narrow_phase: ~NF,
             solver:       ~CS) -> World<RB, I, NF, BP, CD, CS, T>
  {
    World { bodies:       ~[],
            integrator:   integrator,
            broad_phase:  broad_phase,
            narrow_phase: narrow_phase,
            solver:       solver }
  }

  pub fn step(&mut self, dt: T)
  {
    for self.bodies.each |&b|
    { self.integrator.integrate(dt, b) }
  }

  pub fn add(&mut self, body: @mut RB)
  {
    vec::push(&mut self.bodies, body);
  }
}
