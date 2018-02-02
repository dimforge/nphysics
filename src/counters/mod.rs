use std::fmt::{Display, Formatter, Result};

pub use self::timer::Timer;
pub use self::solver_counters::SolverCounters;
pub use self::stages_counters::StagesCounters;
pub use self::collision_detection_counters::CollisionDetectionCounters;

mod timer;
mod solver_counters;
mod stages_counters;
mod collision_detection_counters;

pub struct Counters {
    enabled: bool,
    step_time: Timer,
    stages: StagesCounters,
    cd: CollisionDetectionCounters,
    solver: SolverCounters,
}

impl Counters {
    pub fn new(enabled: bool) -> Self {
        Counters {
            enabled,
            step_time: Timer::new(),
            stages: StagesCounters::new(),
            cd: CollisionDetectionCounters::new(),
            solver: SolverCounters::new(),
        }
    }

    pub fn enable(&mut self) {
        self.enabled = true;
    }

    pub fn enabled(&self) -> bool {
        self.enabled
    }

    pub fn disable(&mut self) {
        self.enabled = false;
    }

    pub fn step_started(&mut self) {
        if self.enabled {
            self.step_time.start();
        }
    }

    pub fn step_completed(&mut self) {
        if self.enabled {
            self.step_time.pause();
        }
    }

    pub fn set_nconstraints(&mut self, n: usize) {
        self.solver.nconstraints = n;
    }

    pub fn set_ncontact_pairs(&mut self, n: usize) {
        self.cd.ncontact_pairs = n;
    }
}

macro_rules! measure_method {
($started: ident, $stopped: ident, $info: ident.$timer: ident) => {
    impl Counters {
        pub fn $started(&mut self) {
            if self.enabled {
                self.$info.$timer.start()
            }
        }

        pub fn $stopped(&mut self) {
            if self.enabled {
                self.$info.$timer.pause()
            }
        }
    }
}
}

measure_method!(update_started, update_completed, stages.update_time);
measure_method!(
    collision_detection_started,
    collision_detection_completed,
    stages.collision_detection_time
);
measure_method!(
    island_construction_started,
    island_construction_completed,
    stages.island_construction_time
);
measure_method!(solver_started, solver_completed, stages.solver_time);

measure_method!(assembly_started, assembly_completed, solver.assembly_time);
measure_method!(
    resolution_started,
    resolution_completed,
    solver.resolution_time
);
measure_method!(
    position_update_started,
    position_update_completed,
    solver.position_update_time
);
measure_method!(
    broad_phase_started,
    broad_phase_completed,
    cd.broad_phase_time
);
measure_method!(
    narrow_phase_started,
    narrow_phase_completed,
    cd.narrow_phase_time
);

impl Display for Counters {
    fn fmt(&self, f: &mut Formatter) -> Result {
        self.stages.fmt(f)?;
        self.cd.fmt(f)?;
        self.solver.fmt(f)
    }
}
