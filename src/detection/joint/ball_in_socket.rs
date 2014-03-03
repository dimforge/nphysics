use nalgebra::na::Transform;
use ncollide::math::Vector;
use detection::joint::anchor::Anchor;

/// A ball-in-socket joint.
///
/// This is usually used to create ragdolls.
pub struct BallInSocket {
    priv up_to_date: bool,
    priv anchor1:    Anchor<Vector>,
    priv anchor2:    Anchor<Vector>,
}

impl BallInSocket {
    /// Creates a ball-in-socket joint.
    pub fn new(anchor1: Anchor<Vector>, anchor2: Anchor<Vector>) -> BallInSocket {
        BallInSocket {
            up_to_date: false,
            anchor1:    anchor1,
            anchor2:    anchor2
        }
    }

    /// Tells if this joint has been modified by the user.
    pub fn up_to_date(&self) -> bool {
        self.up_to_date
    }

    #[doc(hidden)]
    pub fn update(&mut self) {
        self.up_to_date = true
    }

    /// The first anchor affected by this joint.
    pub fn anchor1<'r>(&'r self) -> &'r Anchor<Vector> {
        &self.anchor1
    }

    /// The second anchor affected by this joint.
    pub fn anchor2<'r>(&'r self) -> &'r Anchor<Vector> {
        &self.anchor2
    }

    /// Sets the the second anchor position.
    ///
    /// The position is expressed in the second attached body’s local coordinates.
    pub fn set_local1(&mut self, local1: Vector) {
        if local1 != self.anchor1.position {
            self.up_to_date = false;
            self.anchor1.position = local1
        }
    }

    /// Sets the the second anchor position.
    ///
    /// The position is expressed in the second attached body’s local coordinates.
    pub fn set_local2(&mut self, local2: Vector) {
        if local2 != self.anchor2.position {
            self.up_to_date = false;
            self.anchor2.position = local2
        }
    }

    /// The first attach point in global coordinates.
    pub fn anchor1_pos(&self) -> Vector {
        match self.anchor1.body {
            Some(ref b) => {
                let bb = b.borrow().borrow();
                bb.get().transform_ref().transform(&self.anchor1.position)
            },
            None => self.anchor1.position.clone()
        }
    }

    /// The second attach point in global coordinates.
    pub fn anchor2_pos(&self) -> Vector {
        match self.anchor2.body {
            Some(ref b) => {
                let bb = b.borrow().borrow();
                bb.get().transform_ref().transform(&self.anchor2.position)
            },
            None => self.anchor2.position.clone()
        }
    }
}
