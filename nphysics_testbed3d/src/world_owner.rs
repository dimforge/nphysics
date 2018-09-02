use nphysics3d::world::World;
use std::ops::Deref;
use std::ops::DerefMut;
use std::sync::Arc;
use std::sync::RwLock;

/// This trait is designed to allow choosing implementation of underlying storing of
/// World: shared between threads or with a single owner.
pub trait WorldOwner {
    fn get<'a: 'b, 'b>(&'a self) -> Box<Deref<Target=World<f32>> + 'b>;
    fn get_mut<'a: 'b, 'b>(&'a mut self) -> Box<DerefMut<Target=World<f32>> + 'b>;
}

impl WorldOwner for World<f32> {
    fn get<'a: 'b, 'b>(&'a self) -> Box<Deref<Target=World<f32>> + 'b> {
        Box::new(self)
    }

    fn get_mut<'a: 'b, 'b>(&'a mut self) -> Box<DerefMut<Target=World<f32>> + 'b> {
        Box::new(self)
    }
}

impl WorldOwner for Arc<RwLock<World<f32>>> {
    fn get<'a: 'b, 'b>(&'a self) -> Box<Deref<Target=World<f32>> + 'b> {
        Box::new(self.read().unwrap())
    }

    fn get_mut<'a: 'b, 'b>(&'a mut self) -> Box<DerefMut<Target=World<f32>> + 'b> {
        Box::new(self.write().unwrap())
    }
}
