#![macro_use]

/// Reserved group id for static rigid bodies.
pub const STATIC_GROUP_ID: usize = 29;
/// Reserved group id for sensors.
pub const SENSOR_GROUP_ID: usize = 28;

macro_rules! collision_groups_wrapper_impl(
    ($t: ident) => (
        impl $t {
            /// The maximum allowed group identifier.
            #[inline]
            pub fn max_group_id() -> usize {
                27
            }

            /// Return the internal, ncollide-compatible, `CollisionGroups`
            #[inline]
            pub fn as_collision_groups(&self) -> &CollisionGroups {
                &self.collision_groups
            }

            /// Adds or removes this entity from the given group.
            #[inline]
            pub fn modify_membership(&mut self, group_id: usize, add: bool) {
                assert!(group_id != STATIC_GROUP_ID, "The static group {} is reserved.", STATIC_GROUP_ID);
                assert!(group_id != SENSOR_GROUP_ID, "The sensor group {} is reserved.", SENSOR_GROUP_ID);
                self.collision_groups.modify_membership(group_id, add);
            }

            /// Adds or removes the given group from this entity whitelist.
            #[inline]
            pub fn modify_whitelist(&mut self, group_id: usize, add: bool) {
                assert!(group_id != STATIC_GROUP_ID, "The static group {} is reserved.", STATIC_GROUP_ID);
                assert!(group_id != SENSOR_GROUP_ID, "The sensor group {} is reserved.", SENSOR_GROUP_ID);
                self.collision_groups.modify_whitelist(group_id, add);
            }

            /// Adds or removes this entity from the given group.
            #[inline]
            pub fn modify_blacklist(&mut self, group_id: usize, add: bool) {
                assert!(group_id != STATIC_GROUP_ID, "The static group {} is reserved.", STATIC_GROUP_ID);
                assert!(group_id != SENSOR_GROUP_ID, "The sensor group {} is reserved.", SENSOR_GROUP_ID);
                self.collision_groups.modify_blacklist(group_id, add);
            }

            /// Make this object member of the given groups only.
            #[inline]
            pub fn set_membership(&mut self, groups: &[usize]) {
                assert!(!groups.contains(&STATIC_GROUP_ID), "The static group {} is reserved.", STATIC_GROUP_ID);
                assert!(!groups.contains(&SENSOR_GROUP_ID), "The sensor group {} is reserved.", SENSOR_GROUP_ID);

                let is_dynamic = self.is_dynamic();
                self.collision_groups.set_membership(groups);
                self.configure_reserved_flags(is_dynamic);
            }

            /// Whitelists the given groups only (others will be un-whitelisted).
            #[inline]
            pub fn set_whitelist(&mut self, groups: &[usize]) {
                assert!(!groups.contains(&STATIC_GROUP_ID), "The static group {} is reserved.", STATIC_GROUP_ID);
                assert!(!groups.contains(&SENSOR_GROUP_ID), "The sensor group {} is reserved.", SENSOR_GROUP_ID);

                let is_dynamic = self.is_dynamic();
                self.collision_groups.set_whitelist(groups);
                self.configure_reserved_flags(is_dynamic);
            }

            /// Blacklists the given groups only (others will be un-blacklisted).
            #[inline]
            pub fn set_blacklist(&mut self, groups: &[usize]) {
                assert!(!groups.contains(&STATIC_GROUP_ID), "The static group {} is reserved.", STATIC_GROUP_ID);
                assert!(!groups.contains(&SENSOR_GROUP_ID), "The sensor group {} is reserved.", SENSOR_GROUP_ID);

                let is_dynamic = self.is_dynamic();
                self.collision_groups.set_blacklist(groups);
                self.configure_reserved_flags(is_dynamic);
            }

            /// Copies the membership of another collision groups.
            #[inline]
            pub fn copy_membership(&mut self, other: &$t) {
                let is_dynamic = self.is_dynamic();
                self.collision_groups.copy_membership(other.as_collision_groups());
                self.configure_reserved_flags(is_dynamic);
            }

            /// Copies the whitelist of another collision groups.
            #[inline]
            pub fn copy_whitelist(&mut self, other: &$t) {
                let is_dynamic = self.is_dynamic();
                self.collision_groups.copy_whitelist(other.as_collision_groups());
                self.configure_reserved_flags(is_dynamic);
            }

            /// Copies the blacklist of another collision groups.
            #[inline]
            pub fn copy_blacklist(&mut self, other: &$t) {
                let is_dynamic = self.is_dynamic();
                self.collision_groups.copy_blacklist(other.as_collision_groups());
                self.configure_reserved_flags(is_dynamic);
            }

            /// Un-blacklists static objects.
            pub fn enable_collision_with_static(&mut self) {
                self.collision_groups.modify_blacklist(STATIC_GROUP_ID, false);
                self.collision_groups.modify_whitelist(STATIC_GROUP_ID, true);
            }

            /// Blacklists any static object.
            pub fn disable_collision_with_static(&mut self) {
                self.collision_groups.modify_blacklist(STATIC_GROUP_ID, true);
                self.collision_groups.modify_whitelist(STATIC_GROUP_ID, false);
            }

            /// Un-blacklists sensors.
            pub fn enable_collision_with_sensors(&mut self) {
                self.collision_groups.modify_blacklist(SENSOR_GROUP_ID, false);
                self.collision_groups.modify_whitelist(SENSOR_GROUP_ID, true);
            }

            /// Blacklists sensors.
            pub fn disable_collision_with_sensors(&mut self) {
                self.collision_groups.modify_blacklist(SENSOR_GROUP_ID, true);
                self.collision_groups.modify_whitelist(SENSOR_GROUP_ID, false);
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
    )
);
