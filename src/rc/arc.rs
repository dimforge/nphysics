#![allow(missing_docs)]

use std::ops;
use std::sync::{Arc, RwLock, RwLockReadGuard, RwLockWriteGuard};

type Inner<T> = RwLock<T>;

pub struct Rc<T>(Arc<Inner<T>>);

pub type Ref<'a, T> = RwLockReadGuard<'a, T>;
pub type RefMut<'a, T> = RwLockWriteGuard<'a, T>;

impl<T> Rc<T> {
    pub fn new(val: T) -> Self {
        Rc(Arc::new(RwLock::new(val)))
    }

    pub fn borrow(&self) -> Ref<T> {
        self.0.read().expect("Failed to borrow value")
    }

    pub fn borrow_mut(&self) -> RefMut<T> {
        self.0.write().expect("Failed to mutably borrow value")
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
