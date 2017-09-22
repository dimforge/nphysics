#![allow(missing_docs)]

use std::{self, ops};
use std::cell::{self, RefCell};

type Inner<T> = RefCell<T>;

pub struct Rc<T>(std::rc::Rc<Inner<T>>);

pub type Ref<'a, T> = cell::Ref<'a, T>;
pub type RefMut<'a, T> = cell::RefMut<'a, T>;

impl<T> Rc<T> {
    pub fn new(val: T) -> Self {
        Rc(std::rc::Rc::new(RefCell::new(val)))
    }

    pub fn borrow(&self) -> Ref<T> {
        self.0.borrow()
    }

    pub fn borrow_mut(&self) -> RefMut<T> {
        self.0.borrow_mut()
    }

    pub fn ptr(&self) -> *const Inner<T> {
        &*self.0 as *const _
    }
}

impl<T> Clone for Rc<T> {
    fn clone(&self) -> Self {
        Rc(self.0.clone())
    }
}

impl<T> ops::Deref for Rc<T> {
    type Target = Inner<T>;

    fn deref(&self) -> &Self::Target {
        &*self.0
    }
}
