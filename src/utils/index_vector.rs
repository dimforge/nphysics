// TODO: not very safe as there is no max bound
use std::ops::{Index, IndexMut};
use std::slice::{Iter, IterMut};
use std::iter::FilterMap;

pub trait Indexable {
    fn set_index(&mut self, index: usize);
}

pub type IndexVecIter<'a, T> = FilterMap<Iter<'a, Option<T>>, fn(&Option<T>) -> Option<&T>>;
pub type IndexVecIterMut<'a, T> = FilterMap<IterMut<'a, Option<T>>, fn(&mut Option<T>) -> Option<&mut T>>;

/// Element in this vector never change their position in the vector
pub struct IndexVec<T> {
    data: Vec<Option<T>>,
    empty: Vec<usize>,
    len: usize,
    offset: usize,
}

impl<T> IndexVec<T> {
    pub fn new(offset: usize) -> Self {
        IndexVec {
            data: vec!(),
            empty: Vec::new(),
            len: 0,
            offset
        }
    }

    fn take_next_index(&mut self) -> usize {
        self.len += 1;
        if let Some(index) = self.empty.pop() {
            index
        } else {
            self.data.push(None);
            self.data.len() - 1
        }
    }

    pub fn insert(&mut self, data: T) -> usize {
        let index = self.take_next_index();
        self.data[index] = Some(data);
        index + self.offset
    }

    pub fn remove(&mut self, mut index: usize) -> T {
        index -= self.offset;
        self.empty.push(index);
        self.len -= 1;
        self.data[index].take().unwrap()
    }

    pub fn iter(&self) -> IndexVecIter<T> {
        fn fn_as_ref<T>(d: &Option<T>) -> Option<&T> {
            d.as_ref()
        }
        self.data.iter().filter_map(fn_as_ref)
    }

    pub fn iter_mut(&mut self) -> IndexVecIterMut<T> {
        fn fn_as_mut<T>(d: &mut Option<T>) -> Option<&mut T> {
            d.as_mut()
        }
        self.data.iter_mut().filter_map(fn_as_mut)
    }

    #[inline]
    pub fn len(&self) -> usize {
        self.len
    }
}

impl<T: Indexable> IndexVec<T> {
    #[inline]
    pub fn insert_and_set_index(&mut self, mut data: T) -> usize {
        let index = self.take_next_index();
        data.set_index(index + self.offset);
        self.data[index] = Some(data);
        index + self.offset
    }
}

impl<T: Indexable> Index<usize> for IndexVec<T> {
    type Output = T;
    #[inline]
    fn index(&self, index: usize) -> &T {
        self.data[index - self.offset].as_ref().unwrap()
    }
}

impl<T: Indexable> IndexMut<usize> for IndexVec<T> {
    #[inline]
    fn index_mut(&mut self, index: usize) -> &mut T {
        self.data[index - self.offset].as_mut().unwrap()
    }
}
