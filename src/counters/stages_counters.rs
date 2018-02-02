use std::fmt::{Display, Formatter, Result};
use counters::Timer;

pub struct StagesCounters {
    pub update_time: Timer,
    pub collision_detection_time: Timer,
    pub island_construction_time: Timer,
    pub solver_time: Timer,
}

impl StagesCounters {
    pub fn new() -> Self {
        StagesCounters {
            update_time: Timer::new(),
            collision_detection_time: Timer::new(),
            island_construction_time: Timer::new(),
            solver_time: Timer::new(),
        }
    }
}

impl Display for StagesCounters {
    fn fmt(&self, f: &mut Formatter) -> Result {
        writeln!(f, "Update time: {}", self.update_time)?;
        writeln!(
            f,
            "Collision detection time: {}",
            self.collision_detection_time
        )?;
        writeln!(
            f,
            "Island construction time: {}",
            self.island_construction_time
        )?;
        writeln!(f, "Solver time: {}", self.solver_time)
    }
}
