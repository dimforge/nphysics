use std::rc::Rc;
use std::sync::Arc;
use std::cell::RefCell;
use na::{Transformation, Translation, Rotation, Bounded};
use na;
use na::Transform;
use ncollide::bounding_volume::{HasBoundingVolume, BoundingVolume, AABB, HasAABB};
use ncollide::world::CollisionGroups;
use ncollide::inspection::Repr;
use math::{Scalar, Point, Vect, Orientation, Matrix, AngularInertia};
use volumetric::{InertiaTensor, Volumetric};

/// A shared, mutable, rigid body.
pub type RigidBodyHandle = Rc<RefCell<RigidBody>>;

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
pub enum ActivationState {
    /// The rigid body is active with a not-zero energy.
    Active(Scalar),
    /// The rigid body is inactive.
    Inactive,
    /// The rigid body has been removed from the physics engine.
    Deleted
}

impl ActivationState {
    /// The energy accumulated other several frames.
    pub fn energy(&self) -> Scalar {
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
pub struct RigidBody {
    state:                RigidBodyState,
    shape:                Arc<Box<Repr<Point, Matrix>>>, // FIXME: define our own trait.
    local_to_world:       Matrix,
    lin_vel:              Vect,
    ang_vel:              Orientation,
    inv_mass:             Scalar,
    ls_inv_inertia:       AngularInertia,
    inv_inertia:          AngularInertia,
    ls_center_of_mass:    Point,
    center_of_mass:       Point,
    lin_acc:              Vect,
    ang_acc:              Orientation,
    restitution:          Scalar,
    friction:             Scalar,
    index:                isize,
    activation_state:     ActivationState,
    sleep_threshold:      Option<Scalar>,
    lin_acc_scale:        Vect,        // FIXME: find a better way of doing that.
    ang_acc_scale:        Orientation, // FIXME: find a better way of doing that.
    margin:               Scalar,
    collision_groups:     CollisionGroups
}

impl Clone for RigidBody {
    fn clone(&self) -> RigidBody {
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
            restitution:       self.restitution.clone(),
            friction:          self.friction.clone(),
            index:             self.index.clone(),
            activation_state:  self.activation_state.clone(),
            sleep_threshold:   self.sleep_threshold.clone(),
            lin_acc_scale:     self.lin_acc_scale.clone(),
            ang_acc_scale:     self.ang_acc_scale.clone(),
            margin:            self.margin.clone(),
            collision_groups:  self.collision_groups.clone()
        }
    }
}


impl RigidBody {
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
        self.center_of_mass = self.local_to_world.transform(&self.ls_center_of_mass);
    }

    /// Gets a reference to this body's transform.
    #[inline]
    pub fn position(&self) -> &Matrix {
        &self.local_to_world
    }

    /// Gets a reference to this body's shape.
    #[inline]
    pub fn shape_ref(&self) -> &(Repr<Point, Matrix>) {
        &**self.shape
    }

    /// Gets a copy of this body's shared shape.
    #[inline]
    pub fn shape(&self) -> Arc<Box<Repr<Point, Matrix>>> {
        self.shape.clone()
    }

    /// The margin surrounding this object's shape.
    #[inline]
    pub fn margin(&self) -> Scalar {
        self.margin.clone()
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
    pub fn center_of_mass(&self) -> &Point {
        &self.center_of_mass
    }

    /// Gets this body's restitution coefficent.
    ///
    /// The actual restitution coefficient of a contact is computed averaging the two bodies
    /// friction coefficient.
    #[inline]
    pub fn restitution(&self) -> Scalar {
        self.restitution.clone()
    }

    /// Gets this body's friction coefficient.
    ///
    /// The actual friction coefficient of a contact is computed averaging the two bodies friction
    /// coefficient.
    #[inline]
    pub fn friction(&self) -> Scalar {
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
    /// enough time, the rigid body will fall asleep (i-e be "forzen") for performance reasons.
    #[inline]
    pub fn deactivation_threshold(&self) -> Option<Scalar> {
        self.sleep_threshold.clone()
    }

    /// Set the velocity threshold bellow whith the rigid body might be deactivated.
    ///
    /// If None, the object cannot be deactivated.
    /// If the total squared velocity (i-e: v^2 + w^2) falls bellow this threshold for a long
    /// enough time, the rigid body will fall asleep (i-e be "forzen") for performance reasons.
    #[inline]
    pub fn set_deactivation_threshold(&mut self, threshold: Option<Scalar>) {
        self.sleep_threshold = threshold
    }

    #[doc(hidden)]
    #[inline]
    pub fn activation_state(&self) -> &ActivationState {
        &self.activation_state
    }

    #[doc(hidden)]
    #[inline]
    pub fn activate(&mut self, energy: Scalar) {
        if self.activation_state != ActivationState::Deleted {
            self.activation_state = ActivationState::Active(energy);
        }
    }

    /// Creates a new rigid body that can move.
    pub fn new_dynamic<G>(shape: G, density: Scalar, restitution: Scalar, friction: Scalar) -> RigidBody
        where G: Send + Sync + Repr<Point, Matrix> + Volumetric<Scalar, Point, AngularInertia> {
        let props = shape.mass_properties(density);

        RigidBody::new(
            Arc::new(Box::new(shape) as Box<Repr<Point, Matrix>>),
            Some(props),
            restitution,
            friction)
    }

    /// Creates a new rigid body that cannot move.
    pub fn new_static<G>(shape: G, restitution: Scalar, friction: Scalar) -> RigidBody
        where G: Send + Sync + Repr<Point, Matrix> {
        RigidBody::new(
            Arc::new(Box::new(shape) as Box<Repr<Point, Matrix>>),
            None,
            restitution,
            friction)
    }

    /// Creates a new rigid body with a given shape.
    ///
    /// Use this if the shape is shared by multiple rigid bodies.
    /// Set `mass_properties` to `None` if the rigid body is to be static.
    pub fn new(shape:           Arc<Box<Repr<Point, Matrix>>>,
               mass_properties: Option<(Scalar, Point, AngularInertia)>,
               restitution:     Scalar,
               friction:        Scalar)
               -> RigidBody {
        let (inv_mass, center_of_mass, inv_inertia, active, state) =
            match mass_properties {
                None => (na::zero(), na::orig(), na::zero(), ActivationState::Inactive, RigidBodyState::Static),
                Some((mass, com, inertia)) => {
                    if na::is_zero(&mass) {
                        panic!("A dynamic body must not have a zero volume.")
                    }

                    let ii: AngularInertia;
                    
                    match na::inv(&inertia) {
                        Some(i) => ii = i,
                        None    => ii = na::zero()
                    }

                    let _1: Scalar = na::one();

                    (_1 / mass, com, ii, ActivationState::Active(Bounded::max_value()), RigidBodyState::Dynamic)
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
                center_of_mass:    na::orig(),
                lin_acc:           na::zero(),
                ang_acc:           na::zero(),
                friction:          friction,
                restitution:       restitution,
                index:             0,
                activation_state:  active,
                sleep_threshold:   Some(na::cast(0.1f64)),
                lin_acc_scale:     na::one(),
                ang_acc_scale:     na::one(),
                margin:            na::cast(0.04f32), // FIXME: do not hard-code this.
                collision_groups:  CollisionGroups::new()
            };

        res.update_center_of_mass();
        res.update_inertia_tensor();

        res
    }

    /// The collision groups this rigid body is part of.
    #[inline]
    pub fn collision_groups(&self) -> &CollisionGroups {
        &self.collision_groups
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
    pub fn lin_acc_scale(&self) -> Vect {
        self.lin_acc_scale.clone()
    }

    /// Sets the linear acceleration scale of this rigid body.
    #[inline]
    pub fn set_lin_acc_scale(&mut self, scale: Vect) {
        self.lin_acc_scale = scale
    }

    /// Gets the angular acceleration scale of this rigid body.
    #[inline]
    pub fn ang_acc_scale(&self) -> Orientation {
        self.ang_acc_scale.clone()
    }

    /// Sets the angular acceleration scale of this rigid body.
    #[inline]
    pub fn set_ang_acc_scale(&mut self, scale: Orientation) {
        self.ang_acc_scale = scale
    }

    /// Get the linear velocity of this rigid body.
    #[inline]
    pub fn lin_vel(&self) -> Vect {
        self.lin_vel.clone()
    }

    /// Sets the linear velocity of this rigid body.
    #[inline]
    pub fn set_lin_vel(&mut self, lv: Vect) {
        self.lin_vel = lv
    }

    /// Gets the linear acceleration of this rigid body.
    #[inline]
    pub fn lin_acc(&self) -> Vect {
        self.lin_acc
    }

    /// Sets the linear acceleration of this rigid body.
    ///
    /// Note that this might be reset by the physics engine automatically.
    #[inline]
    pub fn set_lin_acc(&mut self, lf: Vect) {
        self.lin_acc = lf * self.lin_acc_scale
    }

    /// Gets the angular velocity of this rigid body.
    #[inline]
    pub fn ang_vel(&self) -> Orientation {
        self.ang_vel.clone()
    }

    /// Sets the angular velocity of this rigid body.
    #[inline]
    pub fn set_ang_vel(&mut self, av: Orientation) {
        self.ang_vel = av
    }

    /// Gets the angular acceleration of this rigid body.
    #[inline]
    pub fn ang_acc(&self) -> Orientation {
        self.ang_acc.clone()
    }

    /// Sets the angular acceleration of this rigid body.
    ///
    /// Note that this might be reset by the physics engine automatically.
    #[inline]
    pub fn set_ang_acc(&mut self, af: Orientation) {
        self.ang_acc = af * self.ang_acc_scale
    }

    /// Gets the inverse mass of this rigid body.
    #[inline]
    pub fn inv_mass(&self) -> Scalar {
        self.inv_mass.clone()
    }

    /// Sets the inverse mass of this rigid body.
    #[inline]
    pub fn set_inv_mass(&mut self, m: Scalar) {
        self.inv_mass = m
    }

    /// Gets the inverse inertia tensor of this rigid body.
    #[inline]
    pub fn inv_inertia(&self) -> &AngularInertia {
        &self.inv_inertia
    }

    /// Sets the inverse inertia tensor of this rigid body.
    ///
    /// Not that this is reset at every update by the physics engine.
    #[inline]
    pub fn set_inv_inertia(&mut self, ii: AngularInertia) {
        self.inv_inertia = ii
    }

    /// Appends a transformation to this rigid body.
    #[inline]
    pub fn append_transformation(&mut self, to_append: &Matrix) {
        self.local_to_world.append_transformation_mut(to_append);

        self.update_center_of_mass();
        self.update_inertia_tensor();
    }

    /// Prepends a transformation to this rigid body.
    #[inline]
    pub fn prepend_transformation(&mut self, to_prepend: &Matrix) {
        self.local_to_world.prepend_transformation_mut(to_prepend);

        self.update_center_of_mass();
        self.update_inertia_tensor();
    }

    /// Sets the transformation of this rigid body.
    #[inline]
    pub fn set_transformation(&mut self, m: Matrix) {
        self.local_to_world = m;

        self.update_center_of_mass();
        self.update_inertia_tensor();
    }

    /// Appends a translation to this rigid body.
    #[inline]
    pub fn append_translation(&mut self, t: &Vect) {
        self.local_to_world.append_translation_mut(t);
        self.update_center_of_mass();
    }

    /// Prepends a translation to this rigid body.
    #[inline]
    pub fn prepend_translation(&mut self, t: &Vect) {
        self.local_to_world.prepend_translation_mut(t);
        self.update_center_of_mass();
    }


    /// Stes the translation of this rigid body.
    #[inline]
    pub fn set_translation(&mut self, t: Vect) {
        self.local_to_world.set_translation(t);

        self.update_center_of_mass();
    }

    /// Appends a rotation to this rigid body.
    #[inline]
    pub fn append_rotation(&mut self, rot: &Orientation) {
        self.local_to_world.append_rotation_mut(rot);

        self.update_center_of_mass();
        self.update_inertia_tensor();
    }

    /// Prepends a rotation to this rigid body.
    #[inline]
    pub fn prepend_rotation(&mut self, rot: &Orientation) {
        self.local_to_world.prepend_rotation_mut(rot);

        self.update_center_of_mass();
        self.update_inertia_tensor();
    }

    /// Sets the rotation of this rigid body.
    #[inline]
    pub fn set_rotation(&mut self, r: Orientation) {
        self.local_to_world.set_rotation(r);

        self.update_center_of_mass();
        self.update_inertia_tensor();
    }
}

impl HasBoundingVolume<AABB<Point>> for RigidBody {
    fn bounding_volume(&self) -> AABB<Point> {
        self.shape.aabb(&self.local_to_world).loosened(self.margin())
    }
}
