//! Miscellaneous utilities.

pub use self::deterministic_state::DeterministicState;
pub use self::generalized_cross::GeneralizedCross;
pub use self::index_mut2::IndexMut2;
pub use self::user_data::UserData;
pub(crate) use self::user_data::UserDataBox;

mod deterministic_state;
mod generalized_cross;
mod index_mut2;
pub mod union_find;
mod user_data;
