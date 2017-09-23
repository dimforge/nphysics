use detection::joint::anchor::Anchor;

// FIXME: this wont be very helpful to mix several joints.
/// Trait implemented by every joint.
pub trait Joint<A> {
    /// The first anchor affected by this joint.
    fn anchor1(&self) -> &Anchor<A>;
    /// The second anchor affected by this joint.
    fn anchor2(&self) -> &Anchor<A>;
}
