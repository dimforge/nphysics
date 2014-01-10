use std::cell::{Ref, RefMut};

pub trait RefTo<T> {
    fn get<'a>(&'a self) -> &'a T;
}

pub trait MutRefTo<T> {
    fn get<'a>(&'a mut self) -> &'a mut T;
}

impl<'a, T> RefTo<T> for &'a T {
    #[inline]
    fn get<'b>(&'b self) -> &'b T {
        &'b **self
    }
}

impl<'a, T> RefTo<T> for Ref<'a, T> {
    #[inline]
    fn get<'b>(&'b self) -> &'b T {
        self.get()
    }
}

impl<'a, T> MutRefTo<T> for &'a mut T {
    #[inline]
    fn get<'b>(&'b mut self) -> &'b mut T {
        &'b mut **self
    }
}

impl<'a, T> MutRefTo<T> for RefMut<'a, T> {
    #[inline]
    fn get<'b>(&'b mut self) -> &'b mut T {
        self.get()
    }
}
