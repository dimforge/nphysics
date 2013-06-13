use std::util;
use ncollide::narrow::collision_detector::CollisionDetector;
use body::can_move::CanMove;
use graph::accumulator::Accumulator;
use graph::collision_graph::{Node, Edge};

// FIXME: use this alias at some other places (in the solver?)?
type Island<RB, C> = ~[(@mut RB, @mut RB, @mut C)];

struct IslandAccumulator<RB, NF, C>
{
  priv contacts:  Island<RB, C>,
  priv collector: ~[@mut C]
}

impl<RB, NF, C> IslandAccumulator<RB, NF, C>
{
  pub fn new() -> IslandAccumulator<RB, NF, C>
  {
    IslandAccumulator {
      contacts:  ~[],
      collector: ~[]
    }
  }
}

impl<RB: CanMove, NF: CollisionDetector<C, RB, RB>, C>
Accumulator<Edge<NF, RB>, Node<RB, NF>, Island<RB, C>>
for IslandAccumulator<RB, NF, C>
{
  fn reset(&mut self)
  { self.contacts.clear(); }

  fn accumulate_from_edge(&mut self, edge: &mut Edge<NF, RB>) -> bool
  {
    let nf = edge.value();
    let b1 = edge.pred().value();
    let b2 = edge.succ().value();

    nf.update(b1, b2);
    nf.colls(&mut self.collector);

    for self.collector.each |&c|
    { self.contacts.push((b1, b2 ,c)) }

    self.collector.clear();

    true
  }

  fn accumulate_from_node(&mut self, _: &mut Node<RB, NF>) -> bool
  { true } // FIXME: if we want to create islands, do not always return true

  fn result(&mut self) -> Island<RB, C>
  {
    let mut old = ~[];

    util::swap(&mut old, &mut self.contacts);

    old
  }
}
