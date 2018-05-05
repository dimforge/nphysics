use counters::Timer;
use std::fmt::{Display, Formatter, Result};

/// Performance counters related to constraints resolution.
pub struct SolverCounters {
    /// Number of constraints generated.
    pub nconstraints: usize,
    /// Number of contacts found.
    pub ncontacts: usize,
    /// Time spent for the resolution of the constraints (force computation).
    pub resolution_time: Timer,
    /// Time spent for the assembly of all the constraints into a linear complentarity problem.
    pub assembly_time: Timer,
    /// Time spent for the update of the position of the bodies.
    pub position_update_time: Timer,
}

impl SolverCounters {
    /// Creates a new counter initialized to zero.
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
