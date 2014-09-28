use std::num::{Zero, Bounded};
use std::rc::Rc;
use std::sync::Arc;
use std::cell::RefCell;
use na::{Transformation, Translation, Rotation};
use na;
use na::Transform;
use ncollide::bounding_volume::{HasBoundingVolume, LooseBoundingVolume, AABB, HasAABB};
use ncollide::geom::Geom;
use ncollide::volumetric::{InertiaTensor, Volumetric};
use ncollide::math::{Scalar, Vect, Orientation, Matrix, AngularInertia};

/// A shared, mutable, rigid body.
pub type RigidBodyHandle = Rc<RefCell<RigidBody>>;

// FIXME: is this still useful (the same information is given by `self.inv_mass.is_zero()` ?
#[deriving(Show, PartialEq, Clone, Encodable, Decodable)]
/// The movement state of a rigid body.
pub enum RigidBodyState {
    /// The rigid body cannot move.
    Static,
    /// The rigid body can move.
    Dynamic
}

#[deriving(Show, PartialEq, Clone, Encodable, Decodable)]
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
            Active(n) => n.clone(),
            Inactive  => na::zero(),
            Deleted   => na::zero()
        }
    }
}

/// The rigid body structure.
///
/// This is the structure describing an object on the physics world.
pub struct RigidBody {
    state:                RigidBodyState,
    geom:                 Arc<Box<Geom + Send + Sync>>,
    local_to_world:       Matrix,
    lin_vel:              Vect,
    ang_vel:              Orientation,
    inv_mass:             Scalar,
    ls_inv_inertia:       AngularInertia,
    inv_inertia:          AngularInertia,
    ls_center_of_mass:    Vect,
    center_of_mass:       Vect,
    lin_acc:              Vect,
    ang_acc:              Orientation,
    restitution:          Scalar,
    friction:             Scalar,
    index:                int,
    activation_state:     ActivationState,
    sleep_threshold:      Option<Scalar>,
    lin_acc_scale:        Vect,        // FIXME: find a better way of doing that.
    ang_acc_scale:        Orientation, // FIXME: find a better way of doing that.
    margin:               Scalar
}

impl Clone for RigidBody {
    fn clone(&self) -> RigidBody {
        RigidBody {
            state:             self.state.clone(),
            geom:              self.geom.clone(),
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
            margin:            self.margin.clone()
        }
    }
}


impl RigidBody {
    #[doc(hidden)]
    #[inline]
    pub fn deactivate(&mut self) {
        self.lin_vel          = na::zero();
        self.ang_vel          = na::zero();
        self.activation_state = Inactive;
    }

    #[doc(hidden)]
    #[inline]
    pub fn delete(&mut self) {
        self.activation_state = Deleted;
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
    pub fn transform_ref<'r>(&'r self) -> &'r Matrix {
        &self.local_to_world
    }

    /// Gets a reference to this body's geometry.
    #[inline]
    pub fn geom_ref<'r>(&'r self) -> &'r Geom {
        &**self.geom
    }

    /// Gets a copy of this body's shared geometry.
    #[inline]
    pub fn geom<'r>(&'r self) -> Arc<Box<Geom + Send + Sync>> {
        self.geom.clone()
    }

    /// The margin surrounding this object's geometry.
    #[inline]
    pub fn margin(&self) -> Scalar {
        self.margin.clone()
    }

    #[doc(hidden)]
    #[inline]
    pub fn index(&self) -> int {
        self.index
    }

    #[doc(hidden)]
    #[inline]
    pub fn set_index(&mut self, id: int) {
        self.index = id
    }

    /// Gets a reference to this body's center of mass.
    #[inline]
    pub fn center_of_mass<'r>(&'r self) -> &'r Vect {
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
        self.activation_state != Inactive
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
    pub fn activation_state<'a>(&'a self) -> &'a ActivationState {
        &self.activation_state
    }

    #[doc(hidden)]
    #[inline]
    pub fn activate(&mut self, energy: Scalar) {
        self.activation_state = Active(energy);
    }

    /// Creates a new rigid body that can move.
    pub fn new_dynamic<G: Send + Sync + Geom + Volumetric>(
                       geom:        G,
                       density:     Scalar,
                       restitution: Scalar,
                       friction:    Scalar)
                       -> RigidBody {
        let props = geom.mass_properties(&density);

        RigidBody::new(
            Arc::new(box geom as Box<Geom + Send + Sync>),
            Some(props),
            restitution,
            friction)
    }

    /// Creates a new rigid body that cannot move.
    pub fn new_static<G: Send + Sync + Geom>(
                      geom:        G,
                      restitution: Scalar,
                      friction:    Scalar)
                      -> RigidBody {
        RigidBody::new(
            Arc::new(box geom as Box<Geom + Send + Sync>),
            None,
            restitution,
            friction)
    }

    /// Creates a new rigid body with a given geometry.
    ///
    /// Use this if the geometry is shared by multiple rigid bodies.
    /// Set `mass_properties` to `None` if the rigid body is to be static.
    pub fn new(geom:            Arc<Box<Geom + Send + Sync>>,
               mass_properties: Option<(Scalar, Vect, AngularInertia)>,
               restitution:     Scalar,
               friction:        Scalar)
               -> RigidBody {
        let (inv_mass, center_of_mass, inv_inertia, active, state) =
            match mass_properties {
                None => (na::zero(), na::zero(), na::zero(), Inactive, Static),
                Some((mass, com, inertia)) => {
                    if mass.is_zero() {
                        fail!("A dynamic body must not have a zero volume.")
                    }

                    let ii: AngularInertia;
                    
                    match na::inv(&inertia) {
                        Some(i) => ii = i,
                        None    => ii = na::zero()
                    }

                    let _1: Scalar = na::one();

                    (_1 / mass, com, ii, Active(Bounded::max_value()), Dynamic)
                },
            };

        let mut res =
            RigidBody {
                state:                state,
                geom:                 geom,
                local_to_world:       na::one(),
                lin_vel:              na::zero(),
                ang_vel:              na::zero(),
                inv_mass:             inv_mass,
                ls_inv_inertia:       inv_inertia.clone(),
                inv_inertia:          inv_inertia,
                ls_center_of_mass:    center_of_mass,
                center_of_mass:       na::zero(),
                lin_acc:              na::zero(),
                ang_acc:              na::zero(),
                friction:             friction,
                restitution:          restitution,
                index:                0,
                activation_state:     active,
                sleep_threshold:      Some(na::one()),
                lin_acc_scale:        na::one(),
                ang_acc_scale:        na::one(),
                margin:               na::cast(0.04f32) // FIXME: do not hard-code this.
            };

        res.update_center_of_mass();
        res.update_inertia_tensor();

        res
    }

    /// Indicates whether this rigid body is static or dynamic.
    #[inline]
    pub fn can_move(&self) -> bool {
        match self.state {
            Dynamic => true,
            _       => false
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
    pub fn inv_inertia<'r>(&'r self) -> &'r AngularInertia {
        &self.inv_inertia
    }

    /// Sets the inverse inertia tensor of this rigid body.
    ///
    /// Not that this is reset at every update by the physics engine.
    #[inline]
    pub fn set_inv_inertia(&mut self, ii: AngularInertia) {
        self.inv_inertia = ii
    }
}

impl Transformation<Matrix> for RigidBody {
    #[inline]
    fn transformation(&self) -> Matrix {
        self.local_to_world.clone()
    }

    #[inline]
    fn inv_transformation(&self) -> Matrix {
        na::inv(&self.local_to_world).unwrap()
    }

    #[inline]
    fn append_transformation(&mut self, to_append: &Matrix) {
        self.local_to_world.append_transformation(to_append);

        self.update_center_of_mass();
        self.update_inertia_tensor();
    }

    #[inline]
    fn append_transformation_cpy(rb: &RigidBody, m: &Matrix) -> RigidBody {
        let mut res = rb.clone();

        res.append_transformation(m);

        res
    }

    #[inline]
    fn prepend_transformation(&mut self, to_prepend: &Matrix) {
        self.local_to_world.prepend_transformation(to_prepend);

        self.update_center_of_mass();
        self.update_inertia_tensor();
    }

    #[inline]
    fn prepend_transformation_cpy(rb: &RigidBody, m: &Matrix) -> RigidBody {
        let mut res = rb.clone();

        res.prepend_transformation(m);

        res
    }

    #[inline]
    fn set_transformation(&mut self, m: Matrix) {
        self.local_to_world = m;

        self.update_center_of_mass();
        self.update_inertia_tensor();
    }
}

// FIXME: implement Transfomable too

impl Translation<Vect> for RigidBody {
    #[inline]
    fn translation(&self) -> Vect {
        self.local_to_world.translation()
    }

    #[inline]
    fn inv_translation(&self) -> Vect {
        self.local_to_world.inv_translation()
    }

    #[inline]
    fn append_translation(&mut self, t: &Vect) {
        self.local_to_world.append_translation(t);
        self.update_center_of_mass();
    }

    #[inline]
    fn append_translation_cpy(rb: &RigidBody, t: &Vect) -> RigidBody {
        let mut res = rb.clone();

        res.append_translation(t);

        res
    }

    #[inline]
    fn prepend_translation(&mut self, t: &Vect) {
        self.local_to_world.prepend_translation(t);
        self.update_center_of_mass();
    }

    #[inline]
    fn prepend_translation_cpy(rb: &RigidBody, t: &Vect) -> RigidBody {
        let mut res = rb.clone();

        res.prepend_translation(t);

        res
    }

    #[inline]
    fn set_translation(&mut self, t: Vect) {
        self.local_to_world.set_translation(t);

        self.update_center_of_mass();
    }
}

impl Rotation<Orientation> for RigidBody {
    #[inline]
    fn rotation(&self) -> Orientation {
        self.local_to_world.rotation()
    }

    #[inline]
    fn inv_rotation(&self) -> Orientation {
        self.local_to_world.inv_rotation()
    }

    #[inline]
    fn append_rotation(&mut self, rot: &Orientation) {
        self.local_to_world.append_rotation(rot);

        self.update_center_of_mass();
        self.update_inertia_tensor();
    }

    #[inline]
    fn append_rotation_cpy(rb: &RigidBody, rot: &Orientation) -> RigidBody {
        let mut res = rb.clone();

        res.append_rotation(rot);

        res
    }

    #[inline]
    fn prepend_rotation(&mut self, rot: &Orientation) {
        self.local_to_world.prepend_rotation(rot);

        self.update_center_of_mass();
        self.update_inertia_tensor();
    }

    #[inline]
    fn prepend_rotation_cpy(rb: &RigidBody, rot: &Orientation) -> RigidBody {
        let mut res = rb.clone();

        res.prepend_rotation(rot);

        res
    }

    #[inline]
    fn set_rotation(&mut self, r: Orientation) {
        self.local_to_world.set_rotation(r);

        self.update_center_of_mass();
        self.update_inertia_tensor();
    }
}

impl HasBoundingVolume<AABB> for RigidBody {
    fn bounding_volume(&self) -> AABB {
        self.geom.aabb(&self.local_to_world).loosened(self.margin())
    }
}

impl HasBoundingVolume<AABB> for Rc<RefCell<RigidBody>> {
    fn bounding_volume(&self) -> AABB {
        let bself = self.borrow();
        bself.geom().aabb(&bself.local_to_world).loosened(bself.margin())
    }
}
