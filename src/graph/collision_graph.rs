use std::managed::{mut_ptr_eq};
use std::ptr;
use std::vec;
use std::hashmap::HashMap;
use graph::accumulator::Accumulator;

pub struct Graph<N, E>
{
  priv timestamp:         uint,
  priv cleanup_timestamp: uint,
  priv nodes:             ~HashMap<uint, @mut Node<N, E>>
}

pub struct Node<N, E>
{
  priv value:          @mut N,
  priv existing_pairs: ~HashMap<uint, @mut Edge<E, N>>,
  priv edges:          ~[@mut Edge<E, N>],
  priv timestamp:      uint,
  priv updated:        bool
}

pub struct Edge<E, N>
{
  priv value:             @mut E,
  priv pred:              @mut Node<N, E>,
  priv succ:              @mut Node<N, E>,
  priv timestamp:         uint,
  priv pred_id:           uint,
  priv succ_id:           uint,
  priv cleanup_timestamp: uint
}

fn keyof<T>(t: @mut T) -> uint
{ ptr::to_unsafe_ptr(t) as uint }

impl<E, N> Edge<E, N>
{
  pub fn new(value: @mut E, pred: @mut Node<N, E>, succ: @mut Node<N, E>) -> Edge<E, N>
  {
    Edge {
      value:             value,
      pred:              pred,
      succ:              succ,
      timestamp:         0,
      pred_id:           0,
      succ_id:           0,
      cleanup_timestamp: 0 
    }
  }

  pub fn value(&self) -> @mut E
  { self.value }

  pub fn pred(&self) -> @mut Node<N, E>
  { self.pred }

  pub fn succ(&self) -> @mut Node<N, E>
  { self.succ }

  pub fn other_node(&self, o: @mut Node<N, E>) -> @mut Node<N, E>
  {
    if (mut_ptr_eq(o, self.pred))
    { self.succ }
    else
    { self.pred }
  }

  pub fn unlink(to_unlink: @mut Edge<E, N>)
  {
    // pred
    let last_pred = to_unlink.pred.edges.pop();

    if (!mut_ptr_eq(last_pred, to_unlink))
    {
      to_unlink.pred.edges[to_unlink.pred_id] = last_pred;

      if (mut_ptr_eq(last_pred.pred, to_unlink.pred))
      { last_pred.pred_id = to_unlink.pred_id; }
      else
      { last_pred.succ_id = to_unlink.pred_id; }
    }

    // succ
    let last_succ = to_unlink.succ.edges.pop();

    if (!mut_ptr_eq(last_succ, to_unlink))
    {
      to_unlink.succ.edges[to_unlink.succ_id] = last_succ;

      if (mut_ptr_eq(last_succ.pred, to_unlink.succ))
      { last_succ.pred_id = to_unlink.succ_id; }
      else
      { last_succ.succ_id = to_unlink.succ_id; }
    }

    to_unlink.succ.existing_pairs.remove(&keyof(to_unlink.pred));
    to_unlink.pred.existing_pairs.remove(&keyof(to_unlink.succ));
  }

 pub fn accumulate<Acc: Accumulator<Edge<E, N>, Node<N, E>, R>, R>
        (&mut self, accumulator: &mut Acc)
 {
   if (accumulator.accumulate_from_edge(self)) // the accumulator can short-cut the accumulation
   {
     if (self.pred.timestamp != self.timestamp)
     {
       self.pred.timestamp = self.timestamp;
       self.pred.accumulate(accumulator);
     }
     if (self.succ.timestamp != self.timestamp)
     {
       self.succ.timestamp = self.timestamp;
       self.succ.accumulate(accumulator);
     }
   }
 }
}

impl<N, E> Node<N, E>
{
  pub fn new(value: @mut N) -> Node<N, E>
  {
    Node {
      value:          value,
      existing_pairs: ~HashMap::new(),
      edges:          ~[],
      timestamp:      0,
      updated:        false
    }
  }

  pub fn value(&self) -> @mut N
  { self.value }

  pub fn edges<'r>(&'r self) -> &'r ~[@mut Edge<E, N>]
  { &'r self.edges }

  pub fn add_edge(@mut self, edge: @mut Edge<E, N>) -> uint
  {
    self.existing_pairs.insert(keyof(edge.other_node(self)), edge);
    vec::push(&mut self.edges, edge);

    return self.edges.len() - 1;
  }

  pub fn accumulate<Acc: Accumulator<Edge<E, N>, Node<N, E>, R>, R>(&mut self, accumulator: &mut Acc)
  {
    // the accumulator can short-cut the accumulation
    if (accumulator.accumulate_from_node(self))
    {
      for self.edges.each |edge|
      {
        if (edge.timestamp != self.timestamp)
        {
          edge.timestamp = self.timestamp;
          edge.accumulate(accumulator);
        }
      }
    }
  }
}
// 
impl<N, E> Graph<N, E>
{
  pub fn new() -> Graph<N, E>
  {
    Graph {
      timestamp:         0,
      cleanup_timestamp: 0,
      nodes:             ~HashMap::new()
    }
  }

  pub fn add_node(&mut self, node_value: @mut N)
  {
    // the node must not exist already
    assert!(self.nodes.find(&keyof(node_value)).is_none());

    let node = @mut Node::new::<N, E>(node_value);
    node.timestamp = self.timestamp - 1;

    self.nodes.insert(keyof(node_value), node);
  }

  pub fn remove_node(&mut self, node_value: @mut N)
  {
    let k = &keyof(node_value);

    { // new scope to avoid self lifetime problems
      let node = self.nodes.find(k).unwrap();

      for node.edges.each |&edge|
      { Edge::unlink(edge); }
    }

    self.nodes.remove(k);
  }

  pub fn prepare_insersion(&mut self, n1: @mut N, n2: @mut N) -> bool
  {
    let node1 = self.nodes.find(&keyof(n1)).unwrap();
    let node2 = self.nodes.find(&keyof(n2)).unwrap();

    let edge = node1.existing_pairs.find(&keyof(*node2));

    node1.updated = true;

    if (edge.is_some())
    {
      edge.unwrap().cleanup_timestamp = self.cleanup_timestamp;
      true
    }
    else
    { false }
  }

  pub fn accumulate<Acc: Accumulator<Edge<E, N>, Node<N, E>, R>, R>(
    &mut self,
    nodes: &[@mut N],
    acc:   &mut Acc) -> ~[R]
  {
    let mut results = ~[];

    self.timestamp += 1;

    for nodes.each |&node|
    {
      acc.reset();

      let graph_node = self.nodes.find(&keyof(node)).unwrap();

      if (graph_node.timestamp != self.timestamp)
      {
        graph_node.timestamp = self.timestamp;
        graph_node.accumulate(acc);
        results.push(acc.result());
      }
    }

    results
  }

  pub fn accumulate_all<Acc: Accumulator<Edge<E, N>, Node<N, E>, R>, R>(
    &mut self,
    acc: &mut Acc) -> ~[R]
  {
    let mut results = ~[];

    self.timestamp += 1;

    for self.nodes.each_value |&node|
    {
      acc.reset();

      if (node.timestamp != self.timestamp)
      {
        node.timestamp = self.timestamp;
        node.accumulate(acc);
        results.push(acc.result());
      }
    }

    results
  }

  pub fn add_edge(&mut self, edgeValue: @mut E, n1: @mut N, n2: @mut N)
  {
    let node1    = *self.nodes.find(&keyof(n1)).unwrap();
    let node2    = *self.nodes.find(&keyof(n2)).unwrap();
    let existing = node1.existing_pairs.find(&keyof(node2));

    node1.updated = true;

    if (existing.is_none())
    {
      // the edge does not exist yet
      let new_edge = @mut Edge::new(edgeValue, node1, node2);

      let id1 = node1.add_edge(new_edge);
      let id2 = node2.add_edge(new_edge);

      new_edge.pred_id           = id1;
      new_edge.succ_id           = id2;
      new_edge.timestamp         = self.timestamp - 1;
      new_edge.cleanup_timestamp = self.cleanup_timestamp;
    }
  }

  pub fn cleanup(&mut self)
  {
    let mut to_remove = ~[];

    for self.nodes.each_value |node|
    {
      if (node.updated)
      {
        node.updated = false;

        for node.edges.each |&edge|
        {
          if (edge.cleanup_timestamp < self.cleanup_timestamp)
          { to_remove.push(edge); }
        }

        for to_remove.each |&edge|
        { Edge::unlink(edge); }

        to_remove.clear();
      }
    }

    self.cleanup_timestamp += 1;
  }
}
