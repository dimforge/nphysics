use nphysics2d::world::World;
use std::sync::Arc;
use std::sync::Mutex;
use std::ops::DerefMut;

/// This trait is designed to allow choosing implementation of underlying storing of World: shared between threads or owned only by WorldOwner.
pub trait WorldOwner {
    fn get_mut<'a: 'b, 'b>(&'a mut self) -> Box<DerefMut<Target = World<f32>> + 'b>;
}

pub struct WorldOwnerExclusive {
    world: World<f32>
}

impl WorldOwner for WorldOwnerExclusive {
    fn get_mut<'a: 'b, 'b>(&'a mut self) -> Box<DerefMut<Target = World<f32>> + 'b> {
        Box::new(&mut self.world)
    }
}

impl From<World<f32>> for WorldOwnerExclusive {
    fn from(world: World<f32>) -> Self {
        WorldOwnerExclusive {world}
    }
}

#[derive(Clone)]
pub struct WorldOwnerShared {
    world: Arc<Mutex<World<f32>>>,
}

impl WorldOwnerShared {
    pub fn new(w: World<f32>) -> WorldOwnerShared {
        WorldOwnerShared {world: Arc::new(Mutex::new(w))}
    }
}

impl WorldOwner for WorldOwnerShared {
    fn get_mut<'a: 'b, 'b>(&'a mut self) -> Box<DerefMut<Target = World<f32>> + 'b> {
        Box::new(self.world.lock().unwrap())
    }
}