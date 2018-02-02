use std::fmt::{Display, Error, Formatter};
use time;

#[derive(Copy, Clone, Debug)]
pub struct Timer {
    time: f64,
    start: Option<f64>,
}

impl Timer {
    // Creates a new timer.
    pub fn new() -> Self {
        Timer {
            time: 0.0,
            start: None,
        }
    }

    // Starts the timer.
    pub fn start(&mut self) {
        self.time = 0.0;
        self.start = Some(time::precise_time_s());
    }

    // Pauses the timer.
    pub fn pause(&mut self) {
        if let Some(start) = self.start {
            self.time += time::precise_time_s() - start;
        }
        self.start = None;
    }

    // Resumes the timer.
    pub fn resume(&mut self) {
        self.start = Some(time::precise_time_s());
    }

    /// The measured time.
    pub fn time(&self) -> f64 {
        self.time
    }
}

impl Display for Timer {
    fn fmt(&self, f: &mut Formatter) -> Result<(), Error> {
        write!(f, "{}s", self.time)
    }
}
