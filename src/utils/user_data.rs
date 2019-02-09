use std::any::Any;


/// Trait to be implemented by user-defined data.
pub trait UserData: Any + Send + Sync {
    /// Clone this trait-object.
    fn clone_boxed(&self) -> Box<UserData>;
    /// Clone as its super-trait trait objects.
    fn to_any(&self) -> Box<Any + Send + Sync>;
    /// Downcast to Any.
    fn as_any(&self) -> &(Any + Send + Sync);
}

impl<T: Clone + Any + Send + Sync> UserData for T {
    #[inline]
    fn clone_boxed(&self) -> Box<UserData> {
        Box::new(self.clone())
    }

    #[inline]
    fn to_any(&self) -> Box<Any + Send + Sync> {
        Box::new(self.clone())
    }

    #[inline]
    fn as_any(&self) -> &(Any + Send + Sync) {
        self
    }
}

// We need this because we must not implement Clone for Box<UserData>
// directly otherwise Box<UserData> would implement UserData too and
// we want to avoid the user mistakenly nesting user-datas.
pub(crate) struct UserDataBox(pub Box<UserData>);

impl Clone for UserDataBox {
    #[inline]
    fn clone(&self) -> Self {
        UserDataBox(self.0.clone_boxed())
    }
}