//! Counters for benchmarking various parts of the physics engine.

use std::fmt::{Display, Formatter, Result};

pub use self::collision_detection_counters::CollisionDetectionCounters;
pub use self::solver_counters::SolverCounters;
pub use self::stages_counters::StagesCounters;
pub use self::timer::Timer;

mod collision_detection_counters;
mod solver_counters;
mod stages_counters;
mod timer;

/// Aggregation of all the performances counters tracked by nphysics.
pub struct Counters {
    enabled: bool,
    step_time: Timer,
    stages: StagesCounters,
    cd: CollisionDetectionCounters,
    solver: SolverCounters,
}

impl Counters {
    /// Create a new set of counters initialized to wero.
    pub fn new(enabled: bool) -> Self {
        Counters {
            enabled,
            step_time: Timer::new(),
            stages: StagesCounters::new(),
            cd: CollisionDetectionCounters::new(),
            solver: SolverCounters::new(),
        }
    }

    /// Enable all the counters.
    pub fn enable(&mut self) {
        self.enabled = true;
    }

    /// Return `true` if the counters are enabled.
    pub fn enabled(&self) -> bool {
        self.enabled
    }

    /// Disable all the counters.
    pub fn disable(&mut self) {
        self.enabled = false;
    }

    /// Notify that the time-step has started.
    pub fn step_started(&mut self) {
        if self.enabled {
            self.step_time.start();
        }
    }

    /// Notfy that the time-step has finished.
    pub fn step_completed(&mut self) {
        if self.enabled {
            self.step_time.pause();
        }
    }

    /// Total time spent for one step of the physics engine.
    pub fn step_time(&self) -> f64 {
        self.step_time.time()
    }

    /// Set the number of constraints generated.
    pub fn set_nconstraints(&mut self, n: usize) {
        self.solver.nconstraints = n;
    }

    /// Set the number of contact pairs generated.
    pub fn set_ncontact_pairs(&mut self, n: usize) {
        self.cd.ncontact_pairs = n;
    }
}

macro_rules! measure_method {
    ($started:ident, $stopped:ident, $time:ident, $info:ident. $timer:ident) => {
        impl Counters {
            /// Start this timer.
            pub fn $started(&mut self) {
                if self.enabled {
                    self.$info.$timer.start()
                }
            }

            /// Stop this timer.
            pub fn $stopped(&mut self) {
                if self.enabled {
                    self.$info.$timer.pause()
                }
            }

            /// Gets the time elapsed for this timer.
            pub fn $time(&self) -> f64 {
                if self.enabled {
                    self.$info.$timer.time()
                } else {
                    0.0
                }
            }
        }
    };
}

measure_method!(
    update_started,
    update_completed,
    update_time,
    stages.update_time
);
measure_method!(
    collision_detection_started,
    collision_detection_completed,
    collision_detection_time,
    stages.collision_detection_time
);
measure_method!(
    island_construction_started,
    island_construction_completed,
    island_construction_time,
    stages.island_construction_time
);
measure_method!(
    solver_started,
    solver_completed,
    solver_time,
    stages.solver_time
);

measure_method!(
    assembly_started,
    assembly_completed,
    assembly_time,
    solver.assembly_time
);
measure_method!(
    resolution_started,
    resolution_completed,
    resolution_time,
    solver.resolution_time
);
measure_method!(
    position_update_started,
    position_update_completed,
    positon_update_time,
    solver.position_update_time
);
measure_method!(
    broad_phase_started,
    broad_phase_completed,
    broad_phase_time,
    cd.broad_phase_time
);
measure_method!(
    narrow_phase_started,
    narrow_phase_completed,
    narrow_phase_time,
    cd.narrow_phase_time
);

impl Display for Counters {
    fn fmt(&self, f: &mut Formatter) -> Result {
        writeln!(f, "Total timestep time: {}", self.step_time)?;
        self.stages.fmt(f)?;
        self.cd.fmt(f)?;
        self.solver.fmt(f)
    }
}
