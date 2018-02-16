/// An object that has been added to a World.
#[derive(Clone)]
pub enum WorldObject {
    /// A rigid body handle.
    RigidBody(usize),
    /// A sensor handle.
    Sensor(usize)
}

impl WorldObject {
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
    pub fn unwrap_sensor(self) -> usize {
        match self {
            WorldObject::Sensor(s) => s,
            _                      => panic!("This world object is not a sensor.")
        }
    }

    /// Unwraps this object as a rigid body or fails.
    #[inline]
    pub fn unwrap_rigid_body(self) -> usize {
        match self {
            WorldObject::RigidBody(rb) => rb,
            _                          => panic!("This world object is not a rigid body.")
        }
    }

    /// Returns a unique id for this world object.
    #[inline]
    pub fn uid(&self) -> usize {
        match *self {
            WorldObject::RigidBody(uid) => uid,
            WorldObject::Sensor(uid)    => uid,
        }
    }

    // TODO: delete ?
    // #[inline]
    // pub fn position(&self) -> Isometry<N> {
    //     match *self {
    //         WorldObject::RigidBody(ref rb) => rb.position().clone(),
    //         WorldObject::Sensor(ref s)     => s.position()
    //     }
    // }

    // /// A reference to this object geometrical shape.
    // #[inline]
    // pub fn shape(&self) -> &ShapeHandle<Point<N>, Isometry<N>> {
    //     match *self {
    //         WorldObject::RigidBody(ref rb) => rb.shape(),
    //         WorldObject::Sensor(ref s)     => s.shape()
    //     }
    // }

    // /// This object margin.
    // #[inline]
    // pub fn margin(&self) -> N {
    //     match *self {
    //         WorldObject::RigidBody(ref rb) => rb.margin(),
    //         WorldObject::Sensor(ref s)     => s.margin()
    //     }
    // }
}
