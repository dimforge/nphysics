use ncollide::world::CollisionGroups;

// internal reserved static group id
const STATIC_GROUP_ID: usize = 29;

/// Groups of collision used to filter which object collide with which other one.
/// nphysics use a specific group for its own purposes (i.e. the group of static objects).
/// The `group 29` is reserved and you cannot use it.
#[derive(Clone, Debug)]
pub struct RigidBodyCollisionGroups {
    collision_groups: CollisionGroups
}

impl RigidBodyCollisionGroups {
    /// Creates a new `RigidBodyCollisionGroups` that enables collisions with everything except
    /// self-collision.
    #[inline]
    pub fn new_dynamic() -> RigidBodyCollisionGroups {

        // all memberships + all whitelists activated
        let mut groups = CollisionGroups::new();

        groups.modify_membership(STATIC_GROUP_ID, false);
        groups.modify_whitelist(STATIC_GROUP_ID, false);

        RigidBodyCollisionGroups{
            collision_groups: groups
        }
    }

    /// Creates a new `RigidBodyCollisionGroups` that enables collisions with everything except
    /// self-collision.
    #[inline]
    pub fn new_static() -> RigidBodyCollisionGroups {

        // all memberships + all whitelists activated
        let mut groups = CollisionGroups::new();

        groups.modify_whitelist(STATIC_GROUP_ID, false);
        groups.modify_blacklist(STATIC_GROUP_ID, true);

        RigidBodyCollisionGroups{
            collision_groups: groups
        }
    }

    /// Return the internal `CollisionGroups`
    #[inline]
    pub fn as_collision_groups(&self) -> &CollisionGroups {
        &self.collision_groups
    }

    /// Adds or removes this entity from the given group.
    #[inline]
    pub fn modify_membership(&mut self, group_id: usize, add: bool) {
        assert!(group_id != STATIC_GROUP_ID, "The group {} is reserved.", STATIC_GROUP_ID);
        self.collision_groups.modify_membership(group_id, add);
    }

    /// Adds or removes the given group from this entity whitelist.
    #[inline]
    pub fn modify_whitelist(&mut self, group_id: usize, add: bool) {
        assert!(group_id != STATIC_GROUP_ID, "The group {} is reserved.", STATIC_GROUP_ID);
        self.collision_groups.modify_whitelist(group_id, add);
    }

    /// Adds or removes this entity from the given group.
    #[inline]
    pub fn modify_blacklist(&mut self, group_id: usize, add: bool) {
        assert!(group_id != STATIC_GROUP_ID, "The group {} is reserved.", STATIC_GROUP_ID);
        self.collision_groups.modify_blacklist(group_id, add);
    }

    /// Enables self collision detection.
    #[inline]
    pub fn enable_self_collision(&mut self) {
        self.collision_groups.enable_self_collision();
    }

    /// Disables self collision detection.
    #[inline]
    pub fn disable_self_collision(&mut self) {
        self.collision_groups.disable_self_collision();
    }

    /// Tests if this entity is part of the given group.
    #[inline]
    pub fn is_member_of(&self, group_id: usize) -> bool {
        self.collision_groups.is_member_of(group_id)
    }

    /// Tests if the given group is whitelisted.
    #[inline]
    pub fn is_group_whitelisted(&self, group_id: usize) -> bool {
        self.collision_groups.is_group_whitelisted(group_id)
    }

    /// Tests if the given group is blacklisted.
    #[inline]
    pub fn is_group_blacklisted(&self, group_id: usize) -> bool {
        self.collision_groups.is_group_blacklisted(group_id)
    }

    /// Tests whether collisions with a given group is possible.
    ///
    /// Collision is possible if `group_id` is whitelisted but not blacklisted.
    #[inline]
    pub fn can_collide_with(&self, group_id: usize) -> bool {
        self.collision_groups.can_collide_with(group_id)
    }

    /// Tests whether two collision groups have at least one group in common.
    #[inline]
    pub fn can_collide_with_groups(&self, other: &CollisionGroups) -> bool {
        self.collision_groups.can_collide_with_groups(other)
    }

    /// Tests whether self-collision is enabled.
    #[inline]
    pub fn can_collide_with_self(&self) -> bool {
        self.collision_groups.can_collide_with_self()
    }
}
