use std::mem;
use std::any::Any;
use std::rc::Rc;
use std::cell::RefCell;

use alga::general::Real;
use na;
use ncollide::utils::data::hash_map::{HashMap, Entry};
use ncollide::utils::data::hash::UintTWHash;
use ncollide::shape::{Shape, ShapeHandle};
use ncollide::narrow_phase::ProximityHandler;
use ncollide::query::Proximity;
use math::{Point, Isometry};
use object::{RigidBodyHandle, SensorCollisionGroups, WorldObject};
use world::{WorldCollisionObject, RigidBodies, Sensors};

/// A shared, mutable, sensor.
pub type SensorHandle<N> = Rc<RefCell<Sensor<N>>>;

/// An object capable of detecting interferances with other entities without interacting with them.
pub struct Sensor<N: Real> {
    parent:              Option<RigidBodyHandle<N>>,
    relative_position:   Isometry<N>,
    shape:               ShapeHandle<Point<N>, Isometry<N>>,
    margin:              N,
    collision_groups:    SensorCollisionGroups,
    parent_prox:         bool,
    user_data:           Option<Box<Any>>,
    interfering_bodies:  Option<HashMap<usize, RigidBodyHandle<N>, UintTWHash>>,
    interfering_sensors: Option<HashMap<usize, SensorHandle<N>, UintTWHash>>,
    #[doc(hidden)]
    pub did_move_locally:    bool
}

impl<N: Real> Sensor<N> {
    /// Creates a new sensor.
    ///
    /// A sensor may either be attached to the rigid body `parent`, or be attached to the ground.
    /// If it has a parent, it will have a position relative to it. If the parent moves, the sensor
    /// will automatically move as well. By default, a sensor does not trigger any proximity event
    /// when it intersects its own parent.
    ///
    /// A sensor has a default margin equal to zero.
    pub fn new<G>(shape: G, parent: Option<RigidBodyHandle<N>>) -> Sensor<N>
        where G: Send + Sync + Shape<Point<N>, Isometry<N>> {
        Sensor::new_with_shared_shape(ShapeHandle::new(shape), parent)
    }

    /// Creates a new senson with a given shared shape.
    pub fn new_with_shared_shape(shape:  ShapeHandle<Point<N>, Isometry<N>>,
                                 parent: Option<RigidBodyHandle<N>>)
                                 -> Sensor<N> {
        Sensor {
            parent:              parent,
            relative_position:   na::one(),
            did_move_locally:    true,
            shape:               shape,
            margin:              na::zero(),
            collision_groups:    SensorCollisionGroups::new(),
            parent_prox:         false,
            user_data:           None,
            interfering_bodies:  None,
            interfering_sensors: None
        }
    }

    /// Reference to user-defined data attached to this sensor.
    #[inline]
    pub fn user_data(&self) -> Option<&Box<Any>> {
        self.user_data.as_ref()
    }

    /// Mutable reference to user-defined data attached to this sensor.
    #[inline]
    pub fn user_data_mut(&mut self) -> Option<&mut Box<Any>> {
        self.user_data.as_mut()
    }

    /// Attach some user-defined data to this sensor and return the old one.
    pub fn set_user_data(&mut self, user_data: Option<Box<Any>>) -> Option<Box<Any>> {
        mem::replace(&mut self.user_data, user_data)
    }

    /// Enables the collection of all body handles that intersect this sensor.
    pub fn enable_interfering_bodies_collection(&mut self) {
        if self.interfering_bodies.is_none() {
            self.interfering_bodies = Some(HashMap::new(UintTWHash::new()));
        }
    }

    /// Enables the collection of all body handles that intersect this sensor.
    pub fn enable_interfering_sensors_collection(&mut self) {
        if self.interfering_sensors.is_none() {
            self.interfering_sensors= Some(HashMap::new(UintTWHash::new()));
        }
    }

    /// Disables the collection of all body handles that intersect this sensor.
    pub fn disable_interfering_bodies_collection(&mut self) {
        self.interfering_bodies = None;
    }

    /// Disables the collection of all body handles that intersect this sensor.
    pub fn disable_interfering_sensors_collection(&mut self) {
        self.interfering_sensors = None;
    }

    /// List of rigid bodies geometrically intersecting this sensor.
    #[inline]
    pub fn interfering_bodies(&self) -> Option<RigidBodies<N>> {
        // XXX: duplicate code from World.
        // This should actually be a method on the HashMap.
        fn extract_value<N: Real>(e: &Entry<usize, RigidBodyHandle<N>>)
            -> &RigidBodyHandle<N> {
            &e.value
        }

        let extract_value_fn: fn(_) -> _ = extract_value;

        self.interfering_bodies.as_ref().map(|bs| bs.elements().iter().map(extract_value_fn))
    }

    /// List of sensors geometrically intersecting this sensor.
    #[inline]
    pub fn interfering_sensors(&self) -> Option<Sensors<N>> {
        // XXX: duplicate code from World.
        // This should actually be a method on the HashMap.
        fn extract_value<N: Real>(e: &Entry<usize, SensorHandle<N>>)
            -> &SensorHandle<N> {
            &e.value
        }

        let extract_value_fn: fn(_) -> _ = extract_value;
        self.interfering_sensors.as_ref().map(|bs| bs.elements().iter().map(extract_value_fn))
    }

    /// This sensor's position relative to `self.parent()`.
    ///
    /// If this sensor has no parent, then this relative position is actually the sensor absolute
    /// position.
    #[inline]
    pub fn relative_position(&self) -> &Isometry<N> {
        &self.relative_position
    }

    /// Sets the sensor position relative to `self.parent()`.
    ///
    /// If `self.parent()` is `None`, then this sets the sensor's absolute position.
    #[inline]
    pub fn set_relative_position(&mut self, rel_pos: Isometry<N>) {
        self.did_move_locally  = true;
        self.relative_position = rel_pos
    }

    /// This sensor's absolute position.
    #[inline]
    pub fn position(&self) -> Isometry<N> {
        match self.parent {
            Some(ref rb) => *rb.borrow().position() * self.relative_position,
            None         => self.relative_position.clone()
        }
    }

    /// Sets the sensor absolute position.
    ///
    /// If `self.parent()` is not `None`, then this automatically computes the relevant relative
    /// position and updates it.
    #[inline]
    pub fn set_position(&mut self, abs_pos: Isometry<N>) {
        self.did_move_locally = true;
        match self.parent {
            Some(ref rb) => {
                self.relative_position = rb.borrow().position().inverse() * abs_pos
            }
            None => self.relative_position = abs_pos
        }
    }

    /// This sensor position's translational component.
    /// 
    #[inline]
    pub fn center(&self) -> Point<N> {
        match self.parent {
            Some(ref rb) => {
                let coords = self.relative_position.translation.vector;
                *rb.borrow().position() * Point::from_coordinates(coords)
            },
            None => Point::from_coordinates(self.relative_position.translation.vector)
        }
    }

    /// The collision margin of this sensor's shape.
    ///
    /// This is set to zero by default.
    pub fn margin(&self) -> N {
        self.margin
    }

    /// Whether or not proximity detection between this sensor and its parent is enabled.
    #[inline]
    pub fn proximity_with_parent_enabled(&self) -> bool {
        self.parent_prox
    }

    /// Enables proximity detection between this sensor and its parent if it has one.
    #[inline]
    pub fn enable_proximity_with_parent(&mut self) {
        self.parent_prox = true;
    }

    /// Disables proximity detection between this sensor and its parent if it has one.
    #[inline]
    pub fn disable_proximity_with_parent(&mut self) {
        self.parent_prox = false;
    }

    /// The rigid body to which this sensor is kinematically attached.
    ///
    /// Returns `None` if this sensor is directly attached to the scene.
    #[inline]
    pub fn parent(&self) -> Option<&RigidBodyHandle<N>> {
        self.parent.as_ref()
    }

    /// A reference of this sensor's shared shape.
    #[inline]
    pub fn shape(&self) -> &ShapeHandle<Point<N>, Isometry<N>> {
        &self.shape
    }

    /// This sensor's collision groups.
    #[inline]
    pub fn collision_groups(&self) -> &SensorCollisionGroups {
        &self.collision_groups
    }

    /// This sensor's collision groups.
    #[inline]
    pub fn collision_groups_mut(&mut self) -> &mut SensorCollisionGroups {
        &mut self.collision_groups
    }
}

#[doc(hidden)]
pub struct SensorProximityCollector;

impl<N: Real> ProximityHandler<Point<N>, Isometry<N>, WorldObject<N>> for SensorProximityCollector {
    fn handle_proximity(&mut self,
                        o1: &WorldCollisionObject<N>, o2: &WorldCollisionObject<N>,
                        _: Proximity, new_proximity: Proximity) {

        if new_proximity == Proximity::Intersecting {
            match (&o1.data, &o2.data) {
                (&WorldObject::RigidBody(ref rb1), &WorldObject::Sensor(ref s2)) => {
                    let mut s2 = s2.borrow_mut();
                    if let Some(ref mut ib) = s2.interfering_bodies {
                        let _ = ib.insert(WorldObject::rigid_body_uid(rb1), rb1.clone());
                    }
                },
                (&WorldObject::Sensor(ref s1), &WorldObject::RigidBody(ref rb2)) => {
                    let mut s1 = s1.borrow_mut();
                    if let Some(ref mut ib) = s1.interfering_bodies {
                        let _ = ib.insert(WorldObject::rigid_body_uid(rb2), rb2.clone());
                    }
                },
                (&WorldObject::Sensor(ref s1), &WorldObject::Sensor(ref s2)) => {
                    {
                        let mut s1 = s1.borrow_mut();
                        if let Some(ref mut ib) = s1.interfering_sensors {
                            let _ = ib.insert(WorldObject::sensor_uid(s2), s2.clone());
                        }
                    }

                    let mut s2 = s2.borrow_mut();
                    if let Some(ref mut ib) = s2.interfering_sensors {
                        let _ = ib.insert(WorldObject::sensor_uid(s1), s1.clone());
                    }
                },
                (&WorldObject::RigidBody(_), &WorldObject::RigidBody(_)) => { }
            }
        }
        else {
            match (&o1.data, &o2.data) {
                (&WorldObject::RigidBody(ref rb1), &WorldObject::Sensor(ref s2)) => {
                    let mut s2 = s2.borrow_mut();
                    if let Some(ref mut ib) = s2.interfering_bodies {
                        let _ = ib.remove(&WorldObject::rigid_body_uid(rb1));
                    }
                },
                (&WorldObject::Sensor(ref s1), &WorldObject::RigidBody(ref rb2)) => {
                    let mut s1 = s1.borrow_mut();
                    if let Some(ref mut ib) = s1.interfering_bodies {
                        let _ = ib.remove(&WorldObject::rigid_body_uid(rb2));
                    }
                },
                (&WorldObject::Sensor(ref s1), &WorldObject::Sensor(ref s2)) => {
                    {
                        let mut s1 = s1.borrow_mut();
                        if let Some(ref mut ib) = s1.interfering_sensors {
                            let _ = ib.remove(&WorldObject::sensor_uid(s2));
                        }
                    }

                    let mut s2 = s2.borrow_mut();
                    if let Some(ref mut ib) = s2.interfering_sensors {
                        let _ = ib.remove(&WorldObject::sensor_uid(s1));
                    }
                },
                (&WorldObject::RigidBody(_), &WorldObject::RigidBody(_)) => { }
            }
        }
    }
}
