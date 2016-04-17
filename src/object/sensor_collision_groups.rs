use ncollide::world::CollisionGroups;
use object::{STATIC_GROUP_ID, SENSOR_GROUP_ID};

/// Groups of collision used to filter which object collide with which other one.
/// nphysics use a specific group for its own purposes (i.e. the group of static objects).
/// The `group 29` is reserved and you cannot use it.
#[derive(Clone, Debug, Copy)]
pub struct SensorCollisionGroups {
    collision_groups: CollisionGroups
}

impl SensorCollisionGroups {
    /// Creates a new `SensorCollisionGroups` that enables collisions with every user-defined
    /// groups. Static objects and sensors are blacklisted by default and self-collision is disabled.
    #[inline]
    pub fn new() -> SensorCollisionGroups {
        // All memberships + all whitelists activated.
        let mut groups = CollisionGroups::new();

        groups.modify_membership(STATIC_GROUP_ID, false);
        groups.modify_membership(SENSOR_GROUP_ID, true);
        groups.modify_whitelist(STATIC_GROUP_ID,  false);
        groups.modify_whitelist(SENSOR_GROUP_ID,  false);

        groups.modify_blacklist(STATIC_GROUP_ID, true);
        groups.modify_blacklist(SENSOR_GROUP_ID, true);

        SensorCollisionGroups{
            collision_groups: groups
        }
    }


    /*
     * For compatibility with the implementation macro.
     */
    fn configure_reserved_flags(&self, _: bool) {
    }

    fn is_dynamic(&self) -> bool {
        true
    }
}

collision_groups_wrapper_impl!(SensorCollisionGroups);
