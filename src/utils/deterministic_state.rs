use std::hash::SipHasher;
use std::hash::BuildHasher;

/// A hasher builder that creates `SipHasher` with default keys.
pub struct DeterministicState;

impl DeterministicState {
    /// Creates a new `DeterministicState` that builds `SipHasher` with default keys.
    pub fn new() -> DeterministicState {
        DeterministicState
    }
}

impl BuildHasher for DeterministicState {
    type Hasher = SipHasher;

    fn build_hasher(&self) -> SipHasher {
        SipHasher::new()
    }
}
