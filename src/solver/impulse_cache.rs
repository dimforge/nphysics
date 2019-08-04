

use slotmap::SecondaryMap;

use ncollide::query::ContactId;

pub type ImpulseCache<N> = SecondaryMap<ContactId, N>;
