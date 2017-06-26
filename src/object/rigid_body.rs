use std::mem;
use std::any::Any;
use std::ops::Mul;
use num::Bounded;

use alga::general::Real;
use na;
use ncollide::bounding_volume::{self, HasBoundingVolume, BoundingVolume, AABB, BoundingSphere};
use ncollide::shape::{Shape, ShapeHandle};
use utils::GeneralizedCross;
use math::{Point, Vector, Orientation, Rotation, Translation, Isometry, AngularInertia};
use volumetric::{InertiaTensor, Volumetric};
use object::RigidBodyCollisionGroups;

/// A shared, mutable, rigid body.
pub type RigidBodyHandle<N> = ::Rc<RigidBody<N>>;

/// User-defined data attached to a `RigidBody`
pub type UserData = Box<Any + Send + Sync>;

// FIXME: is this still useful (the same information is given by `self.inv_mass.is_zero()` ?
#[derive(Debug, PartialEq, Clone, RustcEncodable, RustcDecodable)]
/// The movement state of a rigid body.
pub enum RigidBodyState { // FIXME: rename this to "RigidBodyKind"?
    /// The rigid body cannot move.
    Static,
    /// The rigid body can move.
    Dynamic
}

#[derive(Debug, PartialEq, Clone, RustcEncodable, RustcDecodable)]
/// The activation state of a rigid body.
pub enum ActivationState<N: Real> {
    /// The rigid body is active with a not-zero energy.
    Active(N),
    /// The rigid body is inactive.
    Inactive,
    /// The rigid body has been removed from the physics engine.
    Deleted
}

impl<N: Real> ActivationState<N> {
    /// The energy accumulated other several frames.
    pub fn energy(&self) -> N {
        match *self {
            ActivationState::Active(n) => n.clone(),
            ActivationState::Inactive  => na::zero(),
            ActivationState::Deleted   => na::zero()
        }
    }
}

/// The rigid body structure.
///
/// This is the structure describing an object on the physics world.
pub struct RigidBody<N: Real> {
    state:                RigidBodyState,
    shape:                ShapeHandle<Point<N>, Isometry<N>>,
    local_to_world:       Isometry<N>,
    lin_vel:              Vector<N>,
    ang_vel:              Orientation<N>,
    inv_mass:             N,
    ls_inv_inertia:       AngularInertia<N>,
    inv_inertia:          AngularInertia<N>,
    ls_center_of_mass:    Point<N>,
    center_of_mass:       Point<N>,
    lin_acc:              Vector<N>,
    ang_acc:              Orientation<N>,
    gravity:              Vector<N>,
    lin_force:            Vector<N>,
    ang_force:            Orientation<N>,
    restitution:          N,
    friction:             N,
    index:                isize,
    activation_state:     ActivationState<N>,
    sleep_threshold:      Option<N>,
    lin_acc_scale:        Vector<N>,      // FIXME: find a better way of doing that.
    ang_acc_scale:        Orientation<N>, // FIXME: find a better way of doing that.
    margin:               N,
    collision_groups:     RigidBodyCollisionGroups,
    user_data:            Option<UserData>
}

impl<N: Real> Clone for RigidBody<N> {
    /// Clones this rigid body but not its associated user-data.
    fn clone(&self) -> RigidBody<N> {
        RigidBody {
            state:             self.state.clone(),
            shape:             self.shape.clone(),
            local_to_world:    self.local_to_world.clone(),
            lin_vel:           self.lin_vel.clone(),
            ang_vel:           self.ang_vel.clone(),
            inv_mass:          self.inv_mass.clone(),
            ls_inv_inertia:    self.ls_inv_inertia.clone(),
            inv_inertia:       self.inv_inertia.clone(),
            ls_center_of_mass: self.ls_center_of_mass.clone(),
            center_of_mass:    self.center_of_mass.clone(),
            lin_acc:           self.lin_acc.clone(),
            ang_acc:           self.ang_acc.clone(),
            gravity:           self.gravity.clone(),
            lin_force:         self.lin_force.clone(),
            ang_force:         self.ang_force.clone(),
            restitution:       self.restitution.clone(),
            friction:          self.friction.clone(),
            index:             self.index.clone(),
            activation_state:  self.activation_state.clone(),
            sleep_threshold:   self.sleep_threshold.clone(),
            lin_acc_scale:     self.lin_acc_scale.clone(),
            ang_acc_scale:     self.ang_acc_scale.clone(),
            margin:            self.margin.clone(),
            collision_groups:  self.collision_groups.clone(),
            user_data:         None
        }
    }
}


impl<N: Real> RigidBody<N> {
    #[doc(hidden)]
    #[inline]
    pub fn deactivate(&mut self) {
        self.lin_vel          = na::zero();
        self.ang_vel          = na::zero();
        self.activation_state = ActivationState::Inactive;
    }

    #[doc(hidden)]
    #[inline]
    pub fn delete(&mut self) {
        self.activation_state = ActivationState::Deleted;
    }

    /// Updates the inertia tensor of this rigid body.
    #[inline]
    fn update_inertia_tensor(&mut self) {
        // FIXME: the inverse inertia should be computed lazily (use a @mut ?).
        self.inv_inertia = self.ls_inv_inertia.to_world_space(&self.local_to_world);
    }

    /// Updates the center of mass of this rigid body.
    #[inline]
    fn update_center_of_mass(&mut self) {
        self.center_of_mass = self.local_to_world * self.ls_center_of_mass;
    }

    /// Gets a reference to this body's transform.
    #[inline]
    pub fn position(&self) -> &Isometry<N> {
        &self.local_to_world
    }

    /// The center given by this object's position. May not be the same as its center of mass.
    ///
    /// This is the point with coordinates `self.position().translation.vector`.
    #[inline]
    pub fn position_center(&self) -> Point<N> {
        Point::from_coordinates(self.local_to_world.translation.vector)
    }

    /// A reference to this body's shape handle.
    #[inline]
    pub fn shape(&self) -> &ShapeHandle<Point<N>, Isometry<N>> {
        &self.shape
    }

    /// The margin surrounding this object's shape.
    #[inline]
    pub fn margin(&self) -> N {
        self.margin.clone()
    }

    /// Sets the margin surrounding this object's shape.
    #[inline]
    pub fn set_margin(&mut self, margin: N) {
        self.margin = margin;
    }

    #[doc(hidden)]
    #[inline]
    pub fn index(&self) -> isize {
        self.index
    }

    #[doc(hidden)]
    #[inline]
    pub fn set_index(&mut self, id: isize) {
        self.index = id
    }

    /// Gets a reference to this body's center of mass.
    #[inline]
    pub fn center_of_mass(&self) -> &Point<N> {
        &self.center_of_mass
    }

    /// Gets this body's mass, if it has one.
    #[inline]
    pub fn mass(&self) -> Option<N> {
        if self.inv_mass == na::zero() {
            None
        } else {
            let _1: N = na::one();
            Some(_1 / self.inv_mass)
        }
    }

    /// Gets this body's restitution coefficent.
    ///
    /// The actual restitution coefficient of a contact is computed averaging the two bodies
    /// restitution coefficient.
    #[inline]
    pub fn restitution(&self) -> N {
        self.restitution.clone()
    }

    /// Gets this body's friction coefficient.
    ///
    /// The actual friction coefficient of a contact is computed averaging the two bodies friction
    /// coefficient.
    #[inline]
    pub fn friction(&self) -> N {
        self.friction.clone()
    }

    /// Indicates whether or not this rigid body is active.
    ///
    /// An inactive rigid body is a body that did not move for some time. It is not longer
    /// simulated by the physics engine as long as no other object touches it. It is automatically
    /// activated by the physics engine.
    #[inline]
    pub fn is_active(&self) -> bool {
        match self.activation_state {
            ActivationState::Active(_) => true,
            _ => false
        }
    }

    /// The velocity threshold bellow whith the rigid body might be deactivated.
    ///
    /// If None, the object cannot be deactivated.
    /// If the total squared velocity (i-e: v^2 + w^2) falls bellow this threshold for a long
    /// enough time, the rigid body will fall asleep (i-e be "frozen") for performance reasons.
    #[inline]
    pub fn deactivation_threshold(&self) -> Option<N> {
        self.sleep_threshold.clone()
    }

    /// Set the velocity threshold bellow whith the rigid body might be deactivated.
    ///
    /// If None, the object cannot be deactivated.
    /// If the total squared velocity (i-e: v^2 + w^2) falls bellow this threshold for a long
    /// enough time, the rigid body will fall asleep (i-e be "frozen") for performance reasons.
    #[inline]
    pub fn set_deactivation_threshold(&mut self, threshold: Option<N>) {
        self.sleep_threshold = threshold
    }

    #[doc(hidden)]
    #[inline]
    pub fn activation_state(&self) -> &ActivationState<N> {
        &self.activation_state
    }

    #[doc(hidden)]
    #[inline]
    pub fn activate(&mut self, energy: N) {
        if self.activation_state != ActivationState::Deleted {
            self.activation_state = ActivationState::Active(energy);
        }
    }

    /// Creates a new rigid body that can move.
    pub fn new_dynamic<G>(shape: G, density: N, restitution: N, friction: N) -> RigidBody<N>
        where G: Send + Sync + Shape<Point<N>, Isometry<N>> + Volumetric<N, Point<N>, AngularInertia<N>> {
        let props = shape.mass_properties(density);

        RigidBody::new(ShapeHandle::new(shape), Some(props), restitution, friction)
    }

    /// Creates a new rigid body that cannot move.
    pub fn new_static<G>(shape: G, restitution: N, friction: N) -> RigidBody<N>
        where G: Send + Sync + Shape<Point<N>, Isometry<N>> {

        RigidBody::new(ShapeHandle::new(shape), None, restitution, friction)
    }

    /// Creates a new rigid body with a given shape.
    ///
    /// Use this if the shape is shared by multiple rigid bodies.
    /// Set `mass_properties` to `None` if the rigid body is to be static.
    pub fn new(shape:           ShapeHandle<Point<N>, Isometry<N>>,
               mass_properties: Option<(N, Point<N>, AngularInertia<N>)>,
               restitution:     N,
               friction:        N)
               -> RigidBody<N> {
        let (inv_mass, center_of_mass, inv_inertia, active, state, groups) =
            match mass_properties {
                None => (na::zero(), na::origin(), na::zero(),
                         ActivationState::Inactive, RigidBodyState::Static,
                         RigidBodyCollisionGroups::new_static()),
                Some((mass, com, inertia)) => {
                    if mass.is_zero() {
                        panic!("A dynamic body must not have a zero volume.")
                    }

                    let ii: AngularInertia<N>;

                    match inertia.try_inverse() {
                        Some(i) => ii = i,
                        None    => ii = na::zero()
                    }

                    // Will be set to a sensible value later.
                    let active = ActivationState::Active(Bounded::max_value());
                    let groups = RigidBodyCollisionGroups::new_dynamic();
                    let _1: N = na::one();

                    (_1 / mass, com, ii, active, RigidBodyState::Dynamic, groups)
                },
            };

        let mut res =
            RigidBody {
                state:             state,
                shape:             shape,
                local_to_world:    na::one(),
                lin_vel:           na::zero(),
                ang_vel:           na::zero(),
                inv_mass:          inv_mass,
                ls_inv_inertia:    inv_inertia.clone(),
                inv_inertia:       inv_inertia,
                ls_center_of_mass: center_of_mass,
                center_of_mass:    na::origin(),
                lin_acc:           na::zero(),
                ang_acc:           na::zero(),
                gravity:           na::zero(),
                lin_force:         na::zero(),
                ang_force:         na::zero(),
                friction:          friction,
                restitution:       restitution,
                index:             0,
                activation_state:  active,
                sleep_threshold:   Some(na::convert(0.1f64)),
                lin_acc_scale:     Vector::from_element(N::one()),
                ang_acc_scale:     Orientation::from_element(N::one()),
                margin:            na::convert(0.04f64), // FIXME: do not hard-code this.
                collision_groups:  groups,
                user_data:         None
            };

        res.update_center_of_mass();
        res.update_inertia_tensor();

        res
    }

    /// Indicates whether this rigid body is static or dynamic.
    #[inline]
    pub fn can_move(&self) -> bool {
        match self.state {
            RigidBodyState::Dynamic => true,
            _ => false
        }
    }

    /// Gets the linear acceleraction scale of this rigid body.
    #[inline]
    pub fn lin_acc_scale(&self) -> Vector<N> {
        self.lin_acc_scale.clone()
    }

    /// Sets the linear acceleration scale of this rigid body.
    #[inline]
    pub fn set_lin_acc_scale(&mut self, scale: Vector<N>) {
        self.lin_acc_scale = scale;
        self.update_lin_acc();
        self.wake_up();
    }

    /// Gets the angular acceleration scale of this rigid body.
    #[inline]
    pub fn ang_acc_scale(&self) -> Orientation<N> {
        self.ang_acc_scale.clone()
    }

    /// Sets the angular acceleration scale of this rigid body.
    #[inline]
    pub fn set_ang_acc_scale(&mut self, scale: Orientation<N>) {
        self.ang_acc_scale = scale;
        self.update_ang_acc();
        self.wake_up();
    }

    /// Get the linear velocity of this rigid body.
    #[inline]
    pub fn lin_vel(&self) -> Vector<N> {
        self.lin_vel.clone()
    }

    /// Sets the linear velocity of this rigid body.
    #[inline]
    pub fn set_lin_vel(&mut self, lv: Vector<N>) {
        self.set_lin_vel_internal(lv);
        self.wake_up();
    }

    /// Sets the linear velocity of this rigid body but does not wake it up.
    #[doc(hidden)]
    #[inline]
    pub fn set_lin_vel_internal(&mut self, lv: Vector<N>) {
        self.lin_vel = lv
    }

    /// Gets the linear acceleration of this rigid body.
    #[inline]
    pub fn lin_acc(&self) -> Vector<N> {
        self.lin_acc
    }

    /// Sets the linear acceleration of this rigid body.
    ///
    /// Note that this might be reset by the physics engine automatically.
    #[doc(hidden)]
    #[inline]
    pub fn set_lin_acc(&mut self, lf: Vector<N>) {
        self.lin_acc = lf
    }

    /// Gets the angular velocity of this rigid body.
    #[inline]
    pub fn ang_vel(&self) -> Orientation<N> {
        self.ang_vel.clone()
    }

    /// Sets the angular velocity of this rigid body.
    #[inline]
    pub fn set_ang_vel(&mut self, av: Orientation<N>) {
        self.set_ang_vel_internal(av);
        self.wake_up();
    }

    /// Sets the angular velocity of this rigid body but does not wake it up.
    #[inline]
    #[doc(hidden)]
    pub fn set_ang_vel_internal(&mut self, av: Orientation<N>) {
        self.ang_vel = av
    }

    /// Gets the angular acceleration of this rigid body.
    #[inline]
    pub fn ang_acc(&self) -> Orientation<N> {
        self.ang_acc.clone()
    }

    /// Sets the angular acceleration of this rigid body.
    ///
    /// Note that this might be reset by the physics engine automatically.
    #[doc(hidden)]
    pub fn set_ang_acc(&mut self, af: Orientation<N>) {
        self.ang_acc = af
    }

    /// Sets the gravity for this RigidBody. It's internally called from BodyForceGenerator,
    /// don't use manually.
    #[doc(hidden)]
    #[inline]
    pub fn set_gravity(&mut self, gravity: Vector<N>) {
        self.gravity = gravity;
        self.update_lin_acc();
    }

    /// Resets linear and angular force.
    #[inline]
    pub fn clear_forces(&mut self) {
        self.clear_linear_force();
        self.clear_angular_force();
    }

    /// Resets linear force.
    #[inline]
    pub fn clear_linear_force(&mut self) {
        self.lin_force = na::zero();
        self.update_lin_acc();
        self.wake_up();
    }

    /// Resets angular force.
    #[inline]
    pub fn clear_angular_force(&mut self) {
        self.ang_force = na::zero();
        self.update_ang_acc();
        self.wake_up();
    }

    /// Adds an additional linear force.
    #[inline]
    pub fn append_lin_force(&mut self, force: Vector<N>) {
        self.lin_force = self.lin_force + force;
        self.update_lin_acc();
        self.wake_up();
    }

    /// Adds an additional angular force.
    #[inline]
    pub fn append_ang_force(&mut self, force: Orientation<N>) {
        self.ang_force = self.ang_force + force;
        self.update_ang_acc();
        self.wake_up();
    }

    /// Adds an additional force acting at a point different to the center of mass.
    ///
    /// The ```pnt_to_com``` vector has to point from the center of mass to
    /// the point where the force acts.
    #[inline]
    pub fn append_force_wrt_point(&mut self, force: Vector<N>, pnt_to_com: Vector<N>) {
        self.append_lin_force(force);
        self.append_ang_force(pnt_to_com.gcross(&force));
    }

    /// Update the linear acceleraction from the applied forces.
    #[inline]
    fn update_lin_acc(&mut self) {
        self.lin_acc = (self.lin_force * self.inv_mass + self.gravity).component_mul(&self.lin_acc_scale);
    }
    /// Update the angular acceleraction from the applied forces.
    #[inline]
    fn update_ang_acc(&mut self) {
        self.ang_acc = (self.inv_inertia * self.ang_force).component_mul(&self.ang_acc_scale);
    }

    /// Forces the body to respond to any impulses before the next tick.
    #[inline]
    pub fn wake_up(&mut self) {
        if self.deactivation_threshold().is_some() {
            self.activate(Bounded::max_value())
        }
    }

    /// Applies a one-time central impulse.
    #[inline]
    pub fn apply_central_impulse(&mut self, impulse: Vector<N>){
        let current_velocity = self.lin_vel();
        let inverted_mass    = self.inv_mass();
        self.set_lin_vel(current_velocity + impulse * inverted_mass);
    }

    /// Applies a one-time angular impulse.
    #[inline]
    pub fn apply_angular_momentum(&mut self, ang_moment: Orientation<N>){
        let current_ang_velocity = self.ang_vel();
        let inverted_tensor = self.inv_inertia().clone();
        self.set_ang_vel(current_ang_velocity + inverted_tensor * ang_moment);
    }

    /// Applies a one-time impulse to a point relative to the center of mass.
    ///
    /// The `pnt_to_com` vector has to point from the center of mass to the
    /// point where the impulse acts.
    #[inline]
    pub fn apply_impulse_wrt_point(&mut self, impulse: Vector<N>, pnt_to_com: Vector<N>){
        self.apply_central_impulse(impulse);
        self.apply_angular_momentum(pnt_to_com.gcross(&impulse));
    }

    /// Gets the inverse mass of this rigid body.
    #[inline]
    pub fn inv_mass(&self) -> N {
        self.inv_mass.clone()
    }

    /// Sets the inverse mass of this rigid body.
    #[inline]
    pub fn set_inv_mass(&mut self, m: N) {
        self.inv_mass = m
    }

    /// Gets the inverse inertia tensor of this rigid body.
    #[inline]
    pub fn inv_inertia(&self) -> &AngularInertia<N> {
        &self.inv_inertia
    }

    /// Sets the inverse inertia tensor of this rigid body.
    ///
    /// Not that this is reset at every update by the physics engine.
    #[inline]
    pub fn set_inv_inertia(&mut self, ii: AngularInertia<N>) {
        self.inv_inertia = ii
    }

    /// Appends a transformation to this rigid body.
    #[inline]
    pub fn append_transformation(&mut self, to_append: &Isometry<N>) {
        self.local_to_world = to_append * self.local_to_world;

        self.update_center_of_mass();
        self.update_inertia_tensor();
    }

    /// Prepends a transformation to this rigid body.
    #[inline]
    pub fn prepend_transformation(&mut self, to_prepend: &Isometry<N>) {
        self.local_to_world *= to_prepend;

        self.update_center_of_mass();
        self.update_inertia_tensor();
    }

    /// Sets the transformation of this rigid body.
    #[inline]
    pub fn set_transformation(&mut self, m: Isometry<N>) {
        self.local_to_world = m;

        self.update_center_of_mass();
        self.update_inertia_tensor();
    }

    /// Appends a translation to this rigid body.
    #[inline]
    pub fn append_translation(&mut self, t: &Translation<N>) {
        self.local_to_world.translation = t * self.local_to_world.translation;
        self.update_center_of_mass();
    }

    /// Prepends a translation to this rigid body.
    #[inline]
    pub fn prepend_translation(&mut self, t: &Translation<N>) {
        self.local_to_world.translation *= t;
        self.update_center_of_mass();
    }


    /// Stes the translation of this rigid body.
    #[inline]
    pub fn set_translation(&mut self, t: Translation<N>) {
        self.local_to_world.translation = t;

        self.update_center_of_mass();
    }

    /// Appends a rotation to this rigid body.
    #[inline]
    pub fn append_rotation(&mut self, rot: &Rotation<N>) {
        self.local_to_world.rotation = rot * self.local_to_world.rotation;

        self.update_center_of_mass();
        self.update_inertia_tensor();
    }

    /// Prepends a rotation to this rigid body.
    #[inline]
    pub fn prepend_rotation(&mut self, rot: &Rotation<N>) {
        self.local_to_world.rotation *= rot;

        self.update_center_of_mass();
        self.update_inertia_tensor();
    }

    /// Sets the rotation of this rigid body.
    #[inline]
    pub fn set_rotation(&mut self, rot: Rotation<N>) {
        self.local_to_world.rotation = rot;

        self.update_center_of_mass();
        self.update_inertia_tensor();
    }

    /// Reference to the collision groups of this rigid body.
    #[inline]
    pub fn collision_groups(&self) -> &RigidBodyCollisionGroups {
        &self.collision_groups
    }

    /// Set the new collisions groups of this rigid body.
    #[inline]
    pub fn set_collision_groups(&mut self, new_groups: RigidBodyCollisionGroups) {
        if self.can_move() && new_groups.is_static() {
            panic!("Attempted to assign collision groups for static rigid bodies to a dynamic rigid body. \
                    TIP: use RigidBodyCollisionGroups::new_dynamic() to create one.");
        }
        if !self.can_move() && new_groups.is_dynamic() {
            panic!("Attempted to assign collision groups for dynamic rigid bodies to a static rigid body. \
                    TIP: use RigidBodyCollisionGroups::new_static() to create one.");
        }

        self.collision_groups = new_groups;
    }

    /// Reference to user-defined data attached to this rigid body.
    #[inline]
    pub fn user_data(&self) -> Option<&UserData> {
        self.user_data.as_ref()
    }

    /// Mutable reference to user-defined data attached to this rigid body.
    #[inline]
    pub fn user_data_mut(&mut self) -> Option<&mut UserData> {
        self.user_data.as_mut()
    }

    /// Attach some user-defined data to this rigid body and return the old one.
    pub fn set_user_data(&mut self, user_data: Option<UserData>) -> Option<UserData> {
        mem::replace(&mut self.user_data, user_data)
    }
}

impl<N, M> HasBoundingVolume<M, BoundingSphere<Point<N>>> for RigidBody<N>
    where N: Real,
          M: Copy + Mul<Isometry<N>, Output = Isometry<N>> { // FIXME: avoiding `Copy` would be great.
    fn bounding_volume(&self, m: &M) -> BoundingSphere<Point<N>> {
        bounding_volume::bounding_sphere(self.shape.as_ref(), &(*m * self.local_to_world)).loosened(self.margin())
    }
}

impl<N, M> HasBoundingVolume<M, AABB<Point<N>>> for RigidBody<N>
    where N: Real,
          M: Copy + Mul<Isometry<N>, Output = Isometry<N>> { // FIXME: avoiding `Copy` would be great.
    fn bounding_volume(&self, m: &M) -> AABB<Point<N>> {
        bounding_volume::aabb(self.shape.as_ref(), &(*m * self.local_to_world)).loosened(self.margin())
    }
}
