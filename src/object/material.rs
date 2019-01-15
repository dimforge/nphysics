use downcast::Any;
use std::sync::Arc;
use std::ops::Deref;
use na::{self, Real};

use ncollide::query::TrackedContact;
use crate::object::{Body, BodyPart, Collider};
use crate::math::Vector;


#[derive(Copy, Clone)]
pub struct MaterialContext<'a, N: Real> {
    pub body: &'a Body<N>,
    pub body_part: &'a BodyPart<N>,
    pub collider: &'a Collider<N>,
    pub contact: &'a TrackedContact<N>,
    pub is_first: bool,
}

impl<'a, N: Real> MaterialContext<'a, N> {
    pub fn new(body: &'a Body<N>, body_part: &'a BodyPart<N>, collider: &'a Collider<N>, contact: &'a TrackedContact<N>, is_first: bool) -> Self {
        MaterialContext {
            body,
            body_part,
            collider,
            contact,
            is_first
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub enum MaterialCombineMode {
    Average,
    Min,
    Multiply,
    Max
}

impl MaterialCombineMode {
    #[inline]
    pub fn combine<N: Real>(a: (N, Self), b: (N, Self)) -> N {
        match (a.1, b.1) {
            (MaterialCombineMode::Max, _) | (_, MaterialCombineMode::Max) => a.0.max(b.0),
            (MaterialCombineMode::Multiply, _) | (_, MaterialCombineMode::Multiply) => a.0 * b.0,
            (MaterialCombineMode::Min, _) | (_, MaterialCombineMode::Min) => a.0.min(b.0),
            // Average
            _ => (a.0 + b.0) * na::convert(0.5)
        }
    }
}

pub trait MaterialClone<N: Real> {
    fn clone_box(&self) -> Box<Material<N>> {
        unimplemented!()
    }
}

impl<N: Real, T: 'static + Material<N> + Clone> MaterialClone<N> for T {
    fn clone_box(&self) -> Box<Material<N>> {
        Box::new(self.clone())
    }
}

pub trait Material<N: Real>: Any + Send + Sync + MaterialClone<N> {
    fn id(&self, context: MaterialContext<N>) -> Option<u8>;
    fn restitution(&self, context: MaterialContext<N>) -> (N, MaterialCombineMode);
    fn friction(&self, context: MaterialContext<N>) -> (N, MaterialCombineMode);
    fn surface_velocity(&self, context: MaterialContext<N>) -> Vector<N>;
}

downcast!(<N> Material<N> where N: Real);

/// Description of the state of surface of a solid.
///
/// Strictly speaking, the coefficient provided here only exist
/// when considering a pair of touching surfaces. In practice, nphysics
/// will average the coefficient of the two surfaces in contact in order
/// to deduce the restitution/friction coefficient.
#[derive(Copy, Clone, Debug)]
pub struct BasicMaterial<N: Real> {
    /// The ID of this material for automatic lookup.
    pub id: Option<u8>,
    /// Restitution coefficient of the surface.
    pub restitution: N,
    /// Friction coefficient of the surface.
    pub friction: N,
    /// The fictitious velocity at the surface of this material.
    pub surface_velocity: Option<Vector<N>>,
    /// The way restitution coefficients are combined if no match
    /// was found in the material lookup tables.
    pub restitution_combine_mode: MaterialCombineMode,
    /// The way friction coefficients are combined if no match
    /// was found in the material lookup tables.
    pub friction_combine_mode: MaterialCombineMode,
}


impl<N: Real> BasicMaterial<N> {
    /// Initialize a material with the specified restitution and friction coefficients.
    pub fn new(restitution: N, friction: N) -> Self {
        BasicMaterial {
            id: None,
            restitution,
            friction,
            surface_velocity: None,
            restitution_combine_mode: MaterialCombineMode::Average,
            friction_combine_mode: MaterialCombineMode::Average
        }
    }
}

impl<N: Real> Material<N> for BasicMaterial<N> {
    fn id(&self, _: MaterialContext<N>) -> Option<u8> {
        self.id
    }

    fn restitution(&self, _: MaterialContext<N>) -> (N, MaterialCombineMode) {
        (self.restitution, self.restitution_combine_mode)
    }

    fn friction(&self, _: MaterialContext<N>) -> (N, MaterialCombineMode) {
        (self.friction, self.friction_combine_mode)
    }

    fn surface_velocity(&self, context: MaterialContext<N>) -> Vector<N> {
        if let Some(ref vel) = self.surface_velocity {
            context.collider.position() * vel
        } else {
            Vector::zeros()
        }
    }
}

impl<N: Real> Default for BasicMaterial<N> {
    fn default() -> Self {
        BasicMaterial::new(N::zero(), na::convert(0.5))
    }
}



impl<N: Real> Clone for Box<Material<N>> {
    fn clone(&self) -> Box<Material<N>> {
        self.clone_box()
    }
}

/// A shared handle to an abstract shape.
///
/// This can be mutated using COW.
#[derive(Clone)]
pub struct MaterialHandle<N: Real>(Arc<Box<Material<N>>>);

impl<N: Real> MaterialHandle<N> {
    /// Creates a sharable shape handle from a shape.
    #[inline]
    pub fn new<S: Material<N> + Clone>(shape: S) -> MaterialHandle<N> {
        MaterialHandle(Arc::new(Box::new(shape)))
    }

    pub(crate) fn make_mut(&mut self) -> &mut Material<N> {
        &mut **Arc::make_mut(&mut self.0)
    }
}

impl<N: Real> AsRef<Material<N>> for MaterialHandle<N> {
    #[inline]
    fn as_ref(&self) -> &Material<N> {
        &*self.deref()
    }
}

impl<N: Real> Deref for MaterialHandle<N> {
    type Target = Material<N>;

    #[inline]
    fn deref(&self) -> &Material<N> {
        &**self.0.deref()
    }
}