use std::mem;
use std::any::Any;
use std::rc::Rc;
use std::cell::RefCell;

use alga::general::Real;
use na;
use ncollide::shape::{Shape, ShapeHandle};
use math::{Point, Isometry};
use object::{RigidBodyHandle, SensorCollisionGroups};

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
    interfering_bodies:  Vec<RigidBodyHandle<N>>,
    interfering_sensors: Vec<SensorHandle<N>>
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
            shape:               shape,
            margin:              na::zero(),
            collision_groups:    SensorCollisionGroups::new(),
            parent_prox:         false,
            user_data:           None,
            interfering_bodies:  Vec::new(),
            interfering_sensors: Vec::new()
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

    /// List of rigid bodies geometrically intersecting this sensor.
    #[inline]
    pub fn interfering_bodies(&self) -> &[RigidBodyHandle<N>] {
        &self.interfering_bodies[..]
    }

    /// List of sensors geometrically intersecting this sensor.
    #[inline]
    pub fn interfering_sensors(&self) -> &[SensorHandle<N>] {
        &self.interfering_sensors[..]
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
}
