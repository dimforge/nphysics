use std::ops::IndexMut;

pub trait IndexMut2<I>: IndexMut<I> {
    fn index_mut2(&mut self, i: usize, j: usize) -> (&mut Self::Output, &mut Self::Output);

    #[inline]
    fn index_mut_const(&mut self, i: usize, j: usize) -> (&mut Self::Output, &Self::Output) {
        let (a, b) = self.index_mut2(i, j);
        (a, &*b)
    }
}

impl<T> IndexMut2<usize> for Vec<T> {
    #[inline]
    fn index_mut2(&mut self, i: usize, j: usize) -> (&mut T, &mut T) {
        assert!(i != j, "Unable to index the same element twice.");
        assert!(i < self.len() && j < self.len(), "Index out of bounds.");

        unsafe {
            let a = &mut *(self.get_unchecked_mut(i) as *mut _);
            let b = &mut *(self.get_unchecked_mut(j) as *mut _);
            (a, b)
        }
    }
}
