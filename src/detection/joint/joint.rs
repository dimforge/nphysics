use alga::general::Real;
use detection::joint::anchor::Anchor;

// FIXME: this wont be very helpful to mix several joints.
/// Trait implemented by every joint.
pub trait Joint<N: Real, A> {
    /// The first anchor affected by this joint.
    fn anchor1(&self) -> &Anchor<N, A>;
    /// The second anchor affected by this joint.
    fn anchor2(&self) -> &Anchor<N, A>;
    /// The first attach point in global coordinates.
    fn anchor1_pos(&self) -> A;
    /// The second attach point in global coordinates.
    fn anchor2_pos(&self) -> A;
}
