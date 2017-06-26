use alga::general::Real;
use ncollide::shape::ShapeHandle;
use object::{RigidBody, Sensor, RigidBodyHandle, SensorHandle};
use math::{Isometry, Point};

/// An object that has been added to a World.
#[derive(Clone)]
pub enum WorldObject<N: Real> {
    /// A rigid body handle.
    RigidBody(RigidBodyHandle<N>),
    /// A sensor handle.
    Sensor(SensorHandle<N>)
}

/// Reference to a world object.
pub enum WorldObjectBorrowed<'a, N: Real> {
    /// A borrowed rigid body handle.
    RigidBody(::Ref<'a, RigidBody<N>>),
    /// A borrowed sensor handle.
    Sensor(::Ref<'a, Sensor<N>>)
}

/// Mutable reference to a world object.
pub enum WorldObjectBorrowedMut<'a, N: Real> {
    /// A mutably borrowed rigid body handle.
    RigidBody(::RefMut<'a, RigidBody<N>>),
    /// A mutably borrowed sensor handle.
    Sensor(::RefMut<'a, Sensor<N>>)
}

impl<N: Real> WorldObject<N> {
    /// The unique identifier a rigid body would have if it was wrapped on a `WorldObject`.
    ///
    /// This identifier remains unique and will not change as long as `rb` is kept alive in memory.
    #[inline]
    pub fn rigid_body_uid(rb: &RigidBodyHandle<N>) -> usize {
        rb.ptr() as usize
    }

    /// The unique identifier a sensor would have if it was wrapped on a `WorldObject`.
    ///
    /// This identifier remains unique and will not change as long as `s` is kept alive in memory.
    #[inline]
    pub fn sensor_uid(s: &SensorHandle<N>) -> usize {
        s.ptr() as usize
    }

    /// Whether or not this is a rigid body.
    #[inline]
    pub fn is_rigid_body(&self) -> bool {
        match *self {
            WorldObject::RigidBody(_) => true,
            _                         => false
        }
    }

    /// Whether or not this is a sensor.
    #[inline]
    pub fn is_sensor(&self) -> bool {
        match *self {
            WorldObject::Sensor(_) => true,
            _                      => false
        }
    }

    /// Unwraps this object as a sensor or fails.
    #[inline]
    pub fn unwrap_sensor(self) -> SensorHandle<N> {
        match self {
            WorldObject::Sensor(s) => s,
            _                      => panic!("This world object is not a sensor.")
        }
    }

    /// Unwraps this object as a rigid body or fails.
    #[inline]
    pub fn unwrap_rigid_body(self) -> RigidBodyHandle<N> {
        match self {
            WorldObject::RigidBody(rb) => rb.clone(),
            _                          => panic!("This world object is not a rigid body.")
        }
    }

    /// Returns a unique id for this world object.
    #[inline]
    pub fn uid(&self) -> usize {
        match *self {
            WorldObject::RigidBody(ref rb) => WorldObject::rigid_body_uid(rb),
            WorldObject::Sensor(ref s)     => WorldObject::sensor_uid(s)
        }
    }

    /// Borrows this world object.
    #[inline]
    pub fn borrow(&self) -> WorldObjectBorrowed<N> {
        match *self {
            WorldObject::RigidBody(ref rb) => WorldObjectBorrowed::RigidBody(rb.borrow()),
            WorldObject::Sensor(ref s)     => WorldObjectBorrowed::Sensor(s.borrow())
        }
    }

    /// Borrows this world object as a sensor.
    #[inline]
    pub fn borrow_sensor(&self) -> ::Ref<Sensor<N>> {
        match *self {
            WorldObject::Sensor(ref s) => s.borrow(),
            _                          => panic!("This world object is not a sensor.")
        }
    }

    /// Borrows this world object as a rigid body.
    #[inline]
    pub fn borrow_rigid_body(&self) -> ::Ref<RigidBody<N>> {
        match *self {
            WorldObject::RigidBody(ref rb) => rb.borrow(),
            _                              => panic!("This world object is not a rigid body.")
        }
    }

    /// Mutably borrows this world object.
    #[inline]
    pub fn borrow_mut(&mut self) -> WorldObjectBorrowedMut<N> {
        match *self {
            WorldObject::RigidBody(ref mut rb) => WorldObjectBorrowedMut::RigidBody(rb.borrow_mut()),
            WorldObject::Sensor(ref mut s)     => WorldObjectBorrowedMut::Sensor(s.borrow_mut())
        }
    }

    /// Mutably borrows this world object as a sensor.
    #[inline]
    pub fn borrow_mut_sensor(&mut self) -> ::RefMut<Sensor<N>> {
        match *self {
            WorldObject::Sensor(ref mut s) => s.borrow_mut(),
            _                              => panic!("This world object is not a sensor.")
        }
    }

    /// Mutably borrows this world object as a rigid body.
    #[inline]
    pub fn borrow_mut_rigid_body(&mut self) -> ::RefMut<RigidBody<N>> {
        match *self {
            WorldObject::RigidBody(ref mut rb) => rb.borrow_mut(),
            _                                  => panic!("This world object is not a rigid body.")
        }
    }
}

macro_rules! impl_getters(
    ($t: ident) => (
        impl<'a, N: Real> $t<'a, N> {
            /// This object's position.
            #[inline]
            pub fn position(&self) -> Isometry<N> {
                match *self {
                    $t::RigidBody(ref rb) => rb.position().clone(),
                    $t::Sensor(ref s)     => s.position()
                }
            }

            /// A reference to this object geometrical shape.
            #[inline]
            pub fn shape(&self) -> &ShapeHandle<Point<N>, Isometry<N>> {
                match *self {
                    $t::RigidBody(ref rb) => rb.shape(),
                    $t::Sensor(ref s)     => s.shape()
                }
            }

            /// This object margin.
            #[inline]
            pub fn margin(&self) -> N {
                match *self {
                    $t::RigidBody(ref rb) => rb.margin(),
                    $t::Sensor(ref s)     => s.margin()
                }
            }

            /// Whether or not this is a rigid body.
            #[inline]
            pub fn is_rigid_body(&self) -> bool {
                match *self {
                    $t::RigidBody(_) => true,
                    _                => false
                }
            }

            /// Whether or not this is a sensor.
            #[inline]
            pub fn is_sensor(&self) -> bool {
                match *self {
                    $t::Sensor(_) => true,
                    _             => false
                }
            }
        }
    )
);

impl_getters!(WorldObjectBorrowed);
impl_getters!(WorldObjectBorrowedMut);
