use std::fmt::{Display, Formatter, Result};
use counters::Timer;

pub struct CollisionDetectionCounters {
    pub ncontact_pairs: usize,
    pub broad_phase_time: Timer,
    pub narrow_phase_time: Timer,
}

impl CollisionDetectionCounters {
    pub fn new() -> Self {
        CollisionDetectionCounters {
            ncontact_pairs:  0,
            broad_phase_time: Timer::new(),
            narrow_phase_time: Timer::new(),
        }
    }
}

impl Display for CollisionDetectionCounters {
    fn fmt(&self, f: &mut Formatter) -> Result {
        writeln!(f, "Number of contact pairs: {}", self.ncontact_pairs)?;
        writeln!(f, "Broad-phase time: {}", self.broad_phase_time)?;
        writeln!(f, "Narrow-phase time: {}", self.narrow_phase_time)
    }
}
