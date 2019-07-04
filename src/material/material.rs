use downcast_rs::Downcast;
use std::sync::Arc;
use std::ops::Deref;
use na::{self, RealField};

use ncollide::query::TrackedContact;
use crate::object::{Body, BodyPart, Collider, BodyHandle, ColliderHandle};
use crate::material::MaterialsCoefficientsTable;
use crate::math::Vector;


/// The context for determining the local material properties at a contact.
#[derive(Copy, Clone)]
pub struct MaterialContext<'a, N: RealField, Handle: BodyHandle> {
    /// One of the two colliders involved in the contact.
    pub collider: &'a Collider<N, Handle>,
    /// The contact.
    pub contact: &'a TrackedContact<N>,
    /// Whether the bodies (and collider) in this structure are the first one involved in the
    /// contact.
    ///
    /// This is `false` if the body involved in the contact is the second one.
    pub is_first: bool,
}

impl<'a, N: RealField, Handle: BodyHandle> MaterialContext<'a, N, Handle> {
    pub(crate) fn new(collider: &'a Collider<N, Handle>, contact: &'a TrackedContact<N>, is_first: bool) -> Self {
        MaterialContext {
            collider,
            contact,
            is_first
        }
    }
}

/// The way the friction and restitution coefficients of two materials should be combined.
#[derive(Copy, Clone, Debug)]
pub enum MaterialCombineMode {
    /// Combination by averaging the coefficients from both materials.
    Average,
    /// Combination by taking the min of the coefficients from both materials.
    ///
    /// Has precedence over the `Average` combine mode.
    Min,
    /// Combination by multiplying the coefficients from both materials.
    ///
    /// Has precedence over the `Min` and `Average` combine modes.
    Multiply,
    /// Combination by taking the max the coefficients from both materials.
    ///
    /// Has precedence over all other combine mode.
    Max,
    /// Should not be used directly. This is set as a result of the `combine` method
    /// if the combination was performed by a lookup on the `MaterialsCoefficientsTable`.
    Lookup // Same as Average if specified by the user.
}

impl MaterialCombineMode {
    /// Combines two coefficients using their associated MaterialCombineMode.
    ///
    /// The combine mode with the highest precedence among the two provided determines
    /// the actual formula used. Precedences are described on the `MaterialCombineMode` enum.
    #[inline]
    pub fn combine<N: RealField>(a: (N, Self), b: (N, Self)) -> (N, MaterialCombineMode) {
        match (a.1, b.1) {
            (MaterialCombineMode::Max, _) | (_, MaterialCombineMode::Max) => (a.0.max(b.0), MaterialCombineMode::Max),
            (MaterialCombineMode::Multiply, _) | (_, MaterialCombineMode::Multiply) => (a.0 * b.0, MaterialCombineMode::Multiply),
            (MaterialCombineMode::Min, _) | (_, MaterialCombineMode::Min) => (a.0.min(b.0), MaterialCombineMode::Min),
            // Average
            _ => ((a.0 + b.0) * na::convert(0.5), MaterialCombineMode::Average)
        }
    }
}

/// Computed material properties at a contact point.
pub struct LocalMaterialProperties<N: RealField> {
    /// The optional material identifier used for pairwise material coefficient lookup table.
    pub id: Option<MaterialId>,
    /// The friction coefficient and its combination mode.
    pub friction: (N, MaterialCombineMode),
    /// The restitution coefficient and its combination mode.
    pub restitution: (N, MaterialCombineMode),
    /// The surface velocity at this point.
    pub surface_velocity: Vector<N>,
}

/// An utility trait to clone material trait-objects.
pub trait MaterialClone<N: RealField, Handle: BodyHandle> {
    /// Clone a material trait-object.
    fn clone_box(&self) -> Box<Material<N, Handle>> {
        unimplemented!()
    }
}

/// The identifier of a material.
pub type MaterialId = u32;

impl<N: RealField, Handle: BodyHandle, T: 'static + Material<N, Handle> + Clone> MaterialClone<N, Handle> for T {
    fn clone_box(&self) -> Box<Material<N, Handle>> {
        Box::new(self.clone())
    }
}

/// An abstract material.
pub trait Material<N: RealField, Handle: BodyHandle>: Downcast + Send + Sync + MaterialClone<N, Handle> {
    /// Retrieve the local material properties of a collider at the given contact point.
    fn local_properties(&self, context: MaterialContext<N, Handle>) -> LocalMaterialProperties<N>;
}

impl_downcast!(Material<N, Handle> where N: RealField, Handle: BodyHandle);

impl<N: RealField, Handle: BodyHandle> Clone for Box<Material<N, Handle>> {
    fn clone(&self) -> Box<Material<N, Handle>> {
        self.clone_box()
    }
}

impl<N: RealField, Handle: BodyHandle> Material<N, Handle> {
    /// Combine two materials given their contexts and a material lookup table.
    pub fn combine<M1, M2>(
        table: &MaterialsCoefficientsTable<N>,
        material1: &M1,
        context1: MaterialContext<N, Handle>,
        material2: &M2,
        context2: MaterialContext<N, Handle>)
        -> LocalMaterialProperties<N>
        where M1: ?Sized + Material<N, Handle>,
              M2: ?Sized + Material<N, Handle> {
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
pub struct MaterialHandle<N: RealField, Handle: BodyHandle>(Arc<Box<Material<N, Handle>>>);

impl<N: RealField, Handle: BodyHandle> MaterialHandle<N, Handle> {
    /// Creates a sharable shape handle from a shape.
    #[inline]
    pub fn new<S: Material<N, Handle> + Clone>(material: S) -> MaterialHandle<N, Handle> {
        MaterialHandle(Arc::new(Box::new(material)))
    }

    pub(crate) fn make_mut(&mut self) -> &mut Material<N, Handle> {
        &mut **Arc::make_mut(&mut self.0)
    }
}

impl<N: RealField, Handle: BodyHandle> AsRef<Material<N, Handle>> for MaterialHandle<N, Handle> {
    #[inline]
    fn as_ref(&self) -> &Material<N, Handle> {
        &*self.deref()
    }
}

impl<N: RealField, Handle: BodyHandle> Deref for MaterialHandle<N, Handle> {
    type Target = Material<N, Handle>;

    #[inline]
    fn deref(&self) -> &Material<N, Handle> {
        &**self.0.deref()
    }
}