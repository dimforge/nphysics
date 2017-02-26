use alga::general::Real;
use math::Point;
use detection::joint::anchor::Anchor;
use detection::joint::joint::Joint;

/// A ball-in-socket joint.
///
/// This is usually used to create ragdolls.
pub struct BallInSocket<N: Real> {
    up_to_date: bool,
    anchor1:    Anchor<N, Point<N>>,
    anchor2:    Anchor<N, Point<N>>,
}

impl<N: Real> BallInSocket<N> {
    /// Creates a ball-in-socket joint.
    pub fn new(anchor1: Anchor<N, Point<N>>, anchor2: Anchor<N, Point<N>>) -> BallInSocket<N> {
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

    /// Sets the the second anchor position.
    ///
    /// The position is expressed in the second attached body’s local coordinates.
    pub fn set_local1(&mut self, local1: Point<N>) {
        if local1 != self.anchor1.position {
            self.up_to_date = false;
            self.anchor1.position = local1
        }
    }

    /// Sets the the second anchor position.
    ///
    /// The position is expressed in the second attached body’s local coordinates.
    pub fn set_local2(&mut self, local2: Point<N>) {
        if local2 != self.anchor2.position {
            self.up_to_date = false;
            self.anchor2.position = local2
        }
    }
}


impl<N: Real> Joint<N, Point<N>> for BallInSocket<N> {
    /// The first anchor affected by this joint.
    #[inline]
    fn anchor1(&self) -> &Anchor<N, Point<N>> {
        &self.anchor1
    }

    /// The second anchor affected by this joint.
    #[inline]
    fn anchor2(&self) -> &Anchor<N, Point<N>> {
        &self.anchor2
    }

    /// The first attach point in global coordinates.
    #[inline]
    fn anchor1_pos(&self) -> Point<N> {
        match self.anchor1.body {
            Some(ref b) => {
                b.borrow().position() * self.anchor1.position
            },
            None => self.anchor1.position.clone()
        }
    }

    /// The second attach point in global coordinates.
    #[inline]
    fn anchor2_pos(&self) -> Point<N> {
        match self.anchor2.body {
            Some(ref b) => {
                b.borrow().position() * self.anchor2.position
            },
            None => self.anchor2.position.clone()
        }
    }
}
