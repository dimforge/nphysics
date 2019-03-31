use downcast_rs::Downcast;
use std::sync::Arc;
use std::ops::Deref;
use na::{self, RealField};

use ncollide::query::TrackedContact;
use crate::object::{Body, BodyPart, Collider};
use crate::material::MaterialsCoefficientsTable;
use crate::math::Vector;


/// The context for determining the local material properties at a contact.
#[derive(Copy, Clone)]
pub struct MaterialContext<'a, N: RealField> {
    /// One of the two bodies involved in the contact.
    pub body: &'a Body<N>,
    /// One of the two bodies part involved in the contact.
    pub body_part: &'a BodyPart<N>,
    /// One of the two colliders involved in the contact.
    pub collider: &'a Collider<N>,
    /// The contact.
    pub contact: &'a TrackedContact<N>,
    /// Whether the bodies (and collider) in this structure are the first one involved in the
    /// contact.
    ///
    /// This is `false` if the body involved in the contact is the second one.
    pub is_first: bool,
}

impl<'a, N: RealField> MaterialContext<'a, N> {
    pub(crate) fn new(body: &'a Body<N>, body_part: &'a BodyPart<N>, collider: &'a Collider<N>, contact: &'a TrackedContact<N>, is_first: bool) -> Self {
        MaterialContext {
            body,
            body_part,
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
pub trait MaterialClone<N: RealField> {
    /// Clone a material trait-object.
    fn clone_box(&self) -> Box<Material<N>> {
        unimplemented!()
    }
}

/// The identifier of a material.
pub type MaterialId = u32;

impl<N: RealField, T: 'static + Material<N> + Clone> MaterialClone<N> for T {
    fn clone_box(&self) -> Box<Material<N>> {
        Box::new(self.clone())
    }
}

/// An abstract material.
pub trait Material<N: RealField>: Downcast + Send + Sync + MaterialClone<N> {
    /// Retrieve the local material properties of a collider at the given contact point.
    fn local_properties(&self, context: MaterialContext<N>) -> LocalMaterialProperties<N>;
}

impl_downcast!(Material<N> where N: RealField);

impl<N: RealField> Clone for Box<Material<N>> {
    fn clone(&self) -> Box<Material<N>> {
        self.clone_box()
    }
}

impl<N: RealField> Material<N> {
    /// Combine two materials given their contexts and a material lookup table.
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
pub struct MaterialHandle<N: RealField>(Arc<Box<Material<N>>>);

impl<N: RealField> MaterialHandle<N> {
    /// Creates a sharable shape handle from a shape.
    #[inline]
    pub fn new<S: Material<N> + Clone>(material: S) -> MaterialHandle<N> {
        MaterialHandle(Arc::new(Box::new(material)))
    }

    pub(crate) fn make_mut(&mut self) -> &mut Material<N> {
        &mut **Arc::make_mut(&mut self.0)
    }
}

impl<N: RealField> AsRef<Material<N>> for MaterialHandle<N> {
    #[inline]
    fn as_ref(&self) -> &Material<N> {
        &*self.deref()
    }
}

impl<N: RealField> Deref for MaterialHandle<N> {
    type Target = Material<N>;

    #[inline]
    fn deref(&self) -> &Material<N> {
        &**self.0.deref()
    }
}