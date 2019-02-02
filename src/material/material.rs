use downcast_rs::Downcast;
use std::sync::Arc;
use std::ops::Deref;
use na::{self, Real};

use ncollide::query::TrackedContact;
use crate::object::{Body, BodyPart, Collider};
use crate::material::MaterialsCoefficientsTable;
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
    Max,
    Lookup // Same as Average if specified by the user.
}

impl MaterialCombineMode {
    #[inline]
    pub fn combine<N: Real>(a: (N, Self), b: (N, Self)) -> (N, MaterialCombineMode) {
        match (a.1, b.1) {
            (MaterialCombineMode::Max, _) | (_, MaterialCombineMode::Max) => (a.0.max(b.0), MaterialCombineMode::Max),
            (MaterialCombineMode::Multiply, _) | (_, MaterialCombineMode::Multiply) => (a.0 * b.0, MaterialCombineMode::Multiply),
            (MaterialCombineMode::Min, _) | (_, MaterialCombineMode::Min) => (a.0.min(b.0), MaterialCombineMode::Min),
            // Average
            _ => ((a.0 + b.0) * na::convert(0.5), MaterialCombineMode::Average)
        }
    }
}

pub struct LocalMaterialProperties<N: Real> {
    pub id: Option<MaterialId>,
    pub friction: (N, MaterialCombineMode),
    pub restitution: (N, MaterialCombineMode),
    pub surface_velocity: Vector<N>,
}

pub trait MaterialClone<N: Real> {
    fn clone_box(&self) -> Box<Material<N>> {
        unimplemented!()
    }
}

pub type MaterialId = u32;

impl<N: Real, T: 'static + Material<N> + Clone> MaterialClone<N> for T {
    fn clone_box(&self) -> Box<Material<N>> {
        Box::new(self.clone())
    }
}

pub trait Material<N: Real>: Downcast + Send + Sync + MaterialClone<N> {
    fn local_properties(&self, context: MaterialContext<N>) -> LocalMaterialProperties<N>;
}

impl_downcast!(Material<N> where N: Real);

impl<N: Real> Clone for Box<Material<N>> {
    fn clone(&self) -> Box<Material<N>> {
        self.clone_box()
    }
}

impl<N: Real> Material<N> {
    pub fn combine<M1, M2>(
        table: &MaterialsCoefficientsTable<N>,
        material1: &M1,
        context1: MaterialContext<N>,
        material2: &M2,
        context2: MaterialContext<N>)
        -> LocalMaterialProperties<N>
        where M1: ?Sized + Material<N>,
              M2: ?Sized + Material<N> {
        let props1 = material1.local_properties(context1);
        let props2 = material2.local_properties(context2);
        let restitution;
        let friction;

        match (props1.id, props2.id) {
            (Some(id1), Some(id2)) => {
                restitution = table.restitution_coefficient(id1, id2)
                    .map(|coeff| (coeff, MaterialCombineMode::Lookup))
                    .unwrap_or_else(|| {
                    MaterialCombineMode::combine(props1.restitution, props2.restitution)
                });
                friction = table.friction_coefficient(id1, id2)
                    .map(|coeff| (coeff, MaterialCombineMode::Lookup))
                    .unwrap_or_else(|| {
                    MaterialCombineMode::combine(props1.friction, props2.friction)
                });
            },
            _ => {
                restitution = MaterialCombineMode::combine(props1.restitution, props2.restitution);
                friction = MaterialCombineMode::combine(props1.friction, props2.friction);
            }
        }

        LocalMaterialProperties {
            id: None,
            friction,
            restitution,
            surface_velocity: props1.surface_velocity - props2.surface_velocity,
        }
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
    pub fn new<S: Material<N> + Clone>(material: S) -> MaterialHandle<N> {
        MaterialHandle(Arc::new(Box::new(material)))
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