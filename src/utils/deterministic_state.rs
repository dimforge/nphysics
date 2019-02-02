use std::collections::hash_map::DefaultHasher;
use std::hash::BuildHasher;

/// A hasher builder that creates `DefaultHasher` with default keys.
#[derive(Default)]
pub struct DeterministicState;

impl DeterministicState {
    /// Creates a new `DeterministicState` that builds `DefaultHasher` with default keys.
    pub fn new() -> Self {
        Self::default()
    }
}

impl BuildHasher for DeterministicState {
    type Hasher = DefaultHasher;

    fn build_hasher(&self) -> DefaultHasher {
        DefaultHasher::new()
    }
}
