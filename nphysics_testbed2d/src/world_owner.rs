use nphysics2d::world::World;
use std::ops::Deref;
use std::ops::DerefMut;
use std::sync::Arc;
use std::sync::RwLock;

/// This trait is designed to allow choosing implementation of underlying storing of World: shared between threads or owned only by WorldOwner.
pub trait WorldOwner {
    fn get<'a: 'b, 'b>(&'a self) -> Box<Deref<Target = World<f32>> + 'b>;
    fn get_mut<'a: 'b, 'b>(&'a mut self) -> Box<DerefMut<Target = World<f32>> + 'b>;
}

pub struct WorldOwnerExclusive {
    world: World<f32>,
}

impl WorldOwner for WorldOwnerExclusive {
    fn get<'a: 'b, 'b>(&'a self) -> Box<Deref<Target = World<f32>> + 'b> {
        Box::new(&self.world)
    }
    fn get_mut<'a: 'b, 'b>(&'a mut self) -> Box<DerefMut<Target = World<f32>> + 'b> {
        Box::new(&mut self.world)
    }
}

impl From<World<f32>> for WorldOwnerExclusive {
    fn from(world: World<f32>) -> Self {
        WorldOwnerExclusive { world }
    }
}

#[derive(Clone)]
pub struct WorldOwnerShared {
    world: Arc<RwLock<World<f32>>>,
}

impl WorldOwnerShared {
    pub fn new(w: World<f32>) -> WorldOwnerShared {
        WorldOwnerShared {
            world: Arc::new(RwLock::new(w)),
        }
    }
}

impl WorldOwner for WorldOwnerShared {
    fn get<'a: 'b, 'b>(&'a self) -> Box<Deref<Target = World<f32>> + 'b> {
        Box::new(self.world.read().unwrap())
    }
    fn get_mut<'a: 'b, 'b>(&'a mut self) -> Box<DerefMut<Target = World<f32>> + 'b> {
        Box::new(self.world.write().unwrap())
    }
}
