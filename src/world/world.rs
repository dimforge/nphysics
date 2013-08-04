use resolution::solver::Solver;
use integration::integrator::Integrator;
use detection::detector::Detector;

pub struct World<N, O, C>
{
  objects:     ~[@mut O],
  integrators: ~[@mut Integrator<N, O>],
  detectors:   ~[@mut Detector<N, O, C>],
  solvers:     ~[@mut Solver<N, C>]
}

impl<N, O, C> World<N, O, C>
{
  pub fn new() -> World<N, O, C>
  {
    World {
      objects:     ~[],
      integrators: ~[],
      detectors:   ~[],
      solvers:     ~[]
    }
  }
}

impl<N: Clone, O, C> World<N, O, C>
{
  pub fn step(&mut self, dt: N)
  {
    //
    // Integration
    //
    for i in self.integrators.mut_iter()
    { i.update(dt.clone()) }

    //
    // Interference detection
    //
    let mut interferences = ~[];

    for d in self.detectors.mut_iter()
    {
      d.update();
      d.interferences(&mut interferences);
    }

    //
    // Resolution
    //
    for s in self.solvers.mut_iter()
    { s.solve(dt.clone(), interferences) }
  }

  pub fn add_detector<D: 'static + Detector<N, O, C>>(&mut self, d: @mut D)
  {
    self.detectors.push(d as @mut Detector<N, O, C>);

    for o in self.objects.iter()
    { d.add(o.clone()) }
  }

  pub fn add_integrator<I: 'static + Integrator<N, O>>(&mut self, i: @mut I)
  {
    self.integrators.push(i as @mut Integrator<N, O>);

    for o in self.objects.mut_iter()
    { i.add(o.clone()) }
  }

  pub fn add_solver<S: 'static + Solver<N, C>>(&mut self, s: @mut S)
  {
      self.solvers.push(s as @mut Solver<N, C>);
  }

  pub fn add_object(&mut self, rb: @mut O)
  {
    self.objects.push(rb);

    let rb = self.objects.last();

    for d in self.integrators.mut_iter()
    { d.add(rb.clone()) }

    for d in self.detectors.mut_iter()
    { d.add(rb.clone()) }
  }
}
