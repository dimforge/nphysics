use ncollide::broad::broad_phase::BroadPhase;
use ncollide::narrow::collision_detector::CollisionDetector;
use graph::collision_graph::Graph;
use graph::island_accumulator::IslandAccumulator;
use integrator::integrator::Integrator;
use constraint::constraint_solver::ConstraintSolver;
use body::can_move::CanMove;

pub struct World<RB, I, NF, BP, CS, C, N>
{
  priv bodies      : ~[@mut RB],
  priv integrator  : ~I,
  priv coll_graph  : ~Graph<RB, NF>,
  priv broad_phase : ~BP,
  priv solver      : ~CS
}

// FIXME: is it good to force CanMove? (no loss of genericity?)
impl<RB: CanMove,
     I : Integrator<N, RB>,
     NF: CollisionDetector<C, RB, RB>,
     BP: BroadPhase<RB>,
     CS: ConstraintSolver<N, RB, C>,
     C,
     N: Copy>
World<RB, I, NF, BP, CS, C, N>
{
  pub fn new(integrator:   ~I,
             broad_phase:  ~BP,
             solver:       ~CS) -> World<RB, I, NF, BP, CS, C, N>
  {
    World { bodies:       ~[],
            integrator:   integrator,
            coll_graph:   ~Graph::new(),
            broad_phase:  broad_phase,
            solver:       solver }
  }

  pub fn step(&mut self, dt: N)
  {
    for self.bodies.iter().advance |&b|
    { self.integrator.integrate(copy dt, b) }

    let pairs = self.broad_phase.collision_pairs(self.bodies);

    for pairs.iter().advance |&(b1, b2)|
    {
      if b1.can_move() || b2.can_move()
      {
        if !self.coll_graph.prepare_insersion(b1, b2)
        { self.coll_graph.add_edge(@mut CollisionDetector::new(b1, b2), b1, b2) }
      }
    }

    self.coll_graph.cleanup();

    let acc = &mut IslandAccumulator::new::<RB, NF, C>();

    // FIXME: do not pass the whole bodies vector: pass only active bodies
    let mut islands = self.coll_graph.accumulate(self.bodies, acc);

    for islands.mut_iter().advance |island|
    { self.solver.solve(copy dt, *island, self.bodies); }
  }

  pub fn add(&mut self, body: @mut RB)
  {
    self.bodies.push(body);
    self.broad_phase.add(body);
    self.coll_graph.add_node(body);
  }

  pub fn coll_graph<'r>(&'r mut self) -> &'r mut ~Graph<RB, NF>
  { &'r mut self.coll_graph }

  pub fn bodies<'r>(&'r self) -> &'r ~[@mut RB]
  { &'r self.bodies }
}
