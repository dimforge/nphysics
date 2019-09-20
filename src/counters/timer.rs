use std::fmt::{Display, Error, Formatter};

/// A timer.
#[derive(Copy, Clone, Debug, Default)]
pub struct Timer {
    time: f64,
    start: Option<f64>,
}

impl Timer {
    /// Creates a new timer initialized to zero and not started.
    pub fn new() -> Self {
        Timer {
            time: 0.0,
            start: None,
        }
    }

    /// Resets the timer to 0.
    pub fn reset(&mut self) {
        self.time = 0.0
    }

    /// Start the timer.
    pub fn start(&mut self) {
        self.time = 0.0;
        self.start = Some(now());
    }

    /// Pause the timer.
    pub fn pause(&mut self) {
        if let Some(start) = self.start {
            self.time += now() - start;
        }
        self.start = None;
    }

    /// Resume the timer.
    pub fn resume(&mut self) {
        self.start = Some(now());
    }

    /// The measured time between the last `.start()` and `.pause()` calls.
    pub fn time(&self) -> f64 {
        self.time
    }
}

#[cfg(not(any(target_arch = "wasm32", target_arch = "asmjs")))]
fn now() -> f64 {
    time::precise_time_s()
}

#[cfg(all(any(target_arch = "wasm32", target_arch = "asmjs"), feature = "stdweb",))]
#[allow(unused_results)] // Needed because the js macro triggers it.
fn now() -> f64 {
    use stdweb::unstable::TryInto;

    // https://developer.mozilla.org/en-US/docs/Web/API/Performance/now
    let v = js! { return performance.now() / 1000.0; };
    v.try_into().unwrap()
}

#[cfg(all(
    any(target_arch = "wasm32", target_arch = "asmjs"),
    feature = "use-wasm-bindgen",
))]
fn now() -> f64 {
    let window = web_sys::window().expect("should have a window in this context");
    let performance = window
        .performance()
        .expect("performance should be available");

    performance.now() / 1000.0
}

impl Display for Timer {
    fn fmt(&self, f: &mut Formatter) -> Result<(), Error> {
        write!(f, "{}s", self.time)
    }
}
