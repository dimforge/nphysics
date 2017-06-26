//! Reference-counted type wrapper
//!
//! Uses `Rc<RefCell<T>>` internally by default, but uses `Arc<RwLock<T>>` instead if the
//! `threadsafe` feature is enabled.

#[cfg(not(feature = "threadsafe"))]
mod rc;
#[cfg(not(feature = "threadsafe"))]
pub use self::rc::*;

#[cfg(feature = "threadsafe")]
mod arc;
#[cfg(feature = "threadsafe")]
pub use self::arc::*;
