use slotmap::SecondaryMap;

use ncollide::query::ContactId;

/// A cache for impulses resulting from contacts and joints.
pub type ImpulseCache<N> = SecondaryMap<ContactId, N>;
