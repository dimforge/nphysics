#![allow(missing_docs)] // for downcast.

use downcast_rs::Downcast;
use slab::Slab;
use na::RealField;

use crate::object::BodySet;
use crate::solver::IntegrationParameters;

pub trait ForceGeneratorSet<N: RealField, Bodies: BodySet<N>> {
    type ForceGenerator: ?Sized + ForceGenerator<N, Bodies>;
//    type Handle: Copy;

    fn get(&self, handle: ForceGeneratorHandle) -> Option<&Self::ForceGenerator>;
    fn get_mut(&mut self, handle: ForceGeneratorHandle) -> Option<&mut Self::ForceGenerator>;

    fn contains(&self, handle: ForceGeneratorHandle) -> bool;

    fn foreach(&self, f: impl FnMut(ForceGeneratorHandle, &Self::ForceGenerator));
    fn foreach_mut(&mut self, f: impl FnMut(ForceGeneratorHandle, &mut Self::ForceGenerator));
}

impl<N: RealField, Bodies: BodySet<N> + 'static> ForceGeneratorSet<N, Bodies> for Slab<Box<ForceGenerator<N, Bodies>>> {
    type ForceGenerator = ForceGenerator<N, Bodies>;
//    type Handle = BodyHandle;

    fn get(&self, handle: ForceGeneratorHandle) -> Option<&Self::ForceGenerator> {
        self.get(handle).map(|c| &**c)
    }

    fn get_mut(&mut self, handle: ForceGeneratorHandle) -> Option<&mut Self::ForceGenerator> {
        self.get_mut(handle).map(|c| &mut **c)
    }

    fn contains(&self, handle: ForceGeneratorHandle) -> bool {
        self.contains(handle)
    }

    fn foreach(&self, mut f: impl FnMut(ForceGeneratorHandle, &Self::ForceGenerator)) {
        for (h, b) in self.iter() {
            f(h, &**b)
        }
    }

    fn foreach_mut(&mut self, mut f: impl FnMut(ForceGeneratorHandle, &mut Self::ForceGenerator)) {
        for (h, b) in self.iter_mut() {
            f(h, &mut **b)
        }
    }
}

/// The handle of a force generator.
pub type ForceGeneratorHandle = usize;

/// A persistent force generator.
/// 
/// A force generator applies a force to one or several bodies at each step of the simulation.
pub trait ForceGenerator<N: RealField, Bodies: BodySet<N>>: Downcast + Send + Sync {
    /// Apply forces to some bodies.
    fn apply(&mut self, params: &IntegrationParameters<N>, bodies: &mut Bodies);
}

impl_downcast!(ForceGenerator<N, Bodies> where N: RealField, Bodies: BodySet<N>);
