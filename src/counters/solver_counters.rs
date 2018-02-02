use std::fmt::{Display, Formatter, Result};
use counters::Timer;

pub struct SolverCounters {
    pub nconstraints: usize,
    pub ncontacts: usize,
    pub resolution_time: Timer,
    pub assembly_time: Timer,
    pub position_update_time: Timer,
}

impl SolverCounters {
    pub fn new() -> Self {
        SolverCounters {
            nconstraints: 0,
            ncontacts: 0,
            resolution_time: Timer::new(),
            assembly_time: Timer::new(),
            position_update_time: Timer::new(),
        }
    }
}

impl Display for SolverCounters {
    fn fmt(&self, f: &mut Formatter) -> Result {
        writeln!(f, "Number of contacts: {}", self.ncontacts)?;
        writeln!(f, "Number of constraints: {}", self.nconstraints)?;
        writeln!(f, "Resolution time: {}", self.resolution_time)?;
        writeln!(f, "Assembly time: {}", self.assembly_time)?;
        writeln!(f, "Position update time: {}", self.position_update_time)
    }
}
