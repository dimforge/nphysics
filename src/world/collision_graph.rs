use std::hashmap::HashMap;
// FIXME: keep this here or move it on ncollide?

struct Graph<N, E>
{
  priv timestamp:         uint,
  priv cleanup_timestamp: uint,
  priv nodes:             ~HashMap<N, Node<N, E>>
}

struct Node<N, E>
{
  priv timestamp:      uint,
  priv existing_pairs: ~HashMap<Node<N, E>, Edge<E, N>>,
  priv edges:          ~[Edge<E, N>],
  priv value:          N,
  priv updated:        bool
}

struct Edge<E, N>
{
  priv value:             E,
  priv pred:              Node<N, E>,
  priv succ:              Node<N, E>,
  priv timestamp:         uint,
  priv prec_id:           uint,
  priv succ_id:           uint,
  priv cleanup_timestamp: uint
}
