use ncollide::world::CollisionGroups;
use object::{SENSOR_GROUP_ID, STATIC_GROUP_ID};

/// Groups of collision used to filter which object collide with which other one.
/// nphysics use a specific group for its own purposes (i.e. the group of static objects).
/// The `group 29` is reserved and you cannot use it.
#[derive(Clone, Debug, Copy)]
pub struct RigidBodyCollisionGroups {
    collision_groups: CollisionGroups,
}

impl RigidBodyCollisionGroups {
    /// Creates a new `RigidBodyCollisionGroups` that enables collisions with everything except
    /// self-collision.
    #[inline]
    pub fn new_dynamic() -> RigidBodyCollisionGroups {
        // All memberships + all whitelists activated.
        let mut groups = CollisionGroups::new();

        groups.modify_membership(STATIC_GROUP_ID, false);
        groups.modify_membership(SENSOR_GROUP_ID, false);
        groups.modify_whitelist(STATIC_GROUP_ID, false);
        groups.modify_whitelist(SENSOR_GROUP_ID, false);

        RigidBodyCollisionGroups {
            collision_groups: groups,
        }
    }

    /// Creates a new `RigidBodyCollisionGroups` that enables collisions with every user-defined
    /// groups. Static objects and sensors are blacklisted by default and self-collision disabled.
    #[inline]
    pub fn new_static() -> RigidBodyCollisionGroups {
        // all memberships + all whitelists activated
        let mut groups = CollisionGroups::new();

        groups.modify_membership(STATIC_GROUP_ID, true);
        groups.modify_membership(SENSOR_GROUP_ID, false);
        groups.modify_whitelist(STATIC_GROUP_ID, false);
        groups.modify_whitelist(SENSOR_GROUP_ID, false);

        groups.modify_blacklist(STATIC_GROUP_ID, true);
        groups.modify_blacklist(SENSOR_GROUP_ID, true);

        RigidBodyCollisionGroups {
            collision_groups: groups,
        }
    }

    fn configure_reserved_flags(&mut self, is_dynamic: bool) {
        if is_dynamic {
            self.collision_groups
                .modify_membership(STATIC_GROUP_ID, false);
            self.collision_groups
                .modify_membership(SENSOR_GROUP_ID, false);
        } else {
            self.collision_groups
                .modify_membership(STATIC_GROUP_ID, true);
            self.collision_groups
                .modify_membership(SENSOR_GROUP_ID, false);
        }
    }

    /// Returns `true` if this object is not part of the static group.
    #[inline]
    pub fn is_dynamic(&self) -> bool {
        !self.is_member_of(STATIC_GROUP_ID)
    }

    /// Returns `true` if this object is part of the static group.
    #[inline]
    pub fn is_static(&self) -> bool {
        self.is_member_of(STATIC_GROUP_ID)
    }
}

collision_groups_wrapper_impl!(RigidBodyCollisionGroups);
