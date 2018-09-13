use std::fmt::{Display, Error, Formatter};

/// A timer.
#[derive(Copy, Clone, Debug)]
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
    use time;
    time::precise_time_s()
}

#[cfg(all(
    any(target_arch = "wasm32", target_arch = "asmjs"),
    feature = "stdweb",
))]
#[allow(unused_results)] // Needed because the js macro triggers it.
fn now() -> f64 {
    use stdweb::unstable::TryInto;

    // https://developer.mozilla.org/en-US/docs/Web/API/Performance/now
    let v = js! { return performance.now() / 1000.0; };
    v.try_into().unwrap()
}
#[cfg(all(
    any(target_arch = "wasm32", target_arch = "asmjs"),
    feature = "wasm-bindgen-no-stdweb",
))]
mod js {
    use wasm_bindgen::prelude::*;
    #[wasm_bindgen]
    extern "C" {
        #[wasm_bindgen(js_namespace = performance)]
        fn now() -> f64;
    }
}

#[cfg(all(
    any(target_arch = "wasm32", target_arch = "asmjs"),
    feature = "wasm-bindgen-no-stdweb",
))]
fn now() -> f64 {
    js::performance::now() / 1000.0
}

impl Display for Timer {
    fn fmt(&self, f: &mut Formatter) -> Result<(), Error> {
        write!(f, "{}s", self.time)
    }
}
