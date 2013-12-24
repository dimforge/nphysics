//! Common trait bouds grouped together.
use std::num::{Zero, One};
use nalgebra::na::{
    Inv,
    Rotation, Rotate, AbsoluteRotate,
    Translation, Transform, Transformation,
    AlgebraicVecExt, Cast, Cross, Indexable,
    Dim
};
use ncollide::volumetric::InertiaTensor;

pub trait NPhysicsScalar: Send           +
                          Freeze         +
                          Zero           +
                          Cast<f32>      +
                          Primitive      +
                          Num            +
                          Algebraic      +
                          Orderable      +
                          Signed         +
                          Real           +
                          ApproxEq<Self> +
                          Float { }

pub trait NPhysicsDirection<N, AV> : Send               +
                                     Freeze             +
                                     Zero               +
                                     AlgebraicVecExt<N> +
                                     Cross<AV>          +
                                     ApproxEq<N>        +
                                     Translation<Self>  +
                                     Rotate<Self>       +
                                     Transform<Self>    +
                                     IterBytes { }

pub trait NPhysicsOrientation<N> : Send   +
                                   Freeze +
                                   Zero   +
                                   AlgebraicVecExt<N> { }

pub trait NPhysicsTransform<LV, AV> : Send                 +
                                      Freeze               +
                                      Inv                  +
                                      Rotation<AV>         +
                                      Rotate<LV>           +
                                      Translation<LV>      +
                                      Transform<LV>        +
                                      Transformation<Self> +
                                      AbsoluteRotate<LV>   +
                                      Mul<Self, Self>      +
                                      Eq                   +
                                      One { }

pub trait NPhysicsInertia<N, LV, AV, M> : Mul<Self, Self>            +
                                          Inv                        +
                                          One                        +
                                          Zero                       +
                                          Indexable<(uint, uint), N> +
                                          Add<Self, Self>            +
                                          Dim                        +
                                          InertiaTensor<N, LV, AV, M> { }

impl<T: Send        +
        Freeze      +
        Zero        +
        Cast<f32>   +
        Primitive   +
        Num         +
        Algebraic   +
        Orderable   +
        Signed      +
        Real        +
        ApproxEq<T> +
        Float>
NPhysicsScalar for T { }

impl<N, AV, T: Send               +
               Freeze             +
               Zero               +
               AlgebraicVecExt<N> +
               Cross<AV>          +
               ApproxEq<N>        +
               Translation<T>     +
               Rotate<T>          +
               Transform<T>       +
               IterBytes>
NPhysicsDirection<N, AV> for T { }

impl<N, T: Send   +
           Freeze +
           Zero   +
           AlgebraicVecExt<N>>
NPhysicsOrientation<N> for T { }

impl<LV, AV, T: Send               +
                Freeze             +
                Inv                +
                Rotation<AV>       +
                Rotate<LV>         +
                Translation<LV>    +
                Transform<LV>      +
                Transformation<T>  +
                AbsoluteRotate<LV> +
                Mul<T, T>          +
                Eq                 +
                One>
NPhysicsTransform<LV, AV> for T { }

impl<N, LV, AV, M, T: Mul<T, T>                  +
                      Inv                        +
                      One                        +
                      Zero                       +
                      Indexable<(uint, uint), N> +
                      Add<T, T>                  +
                      Dim                        +
                      InertiaTensor<N, LV, AV, M>>
NPhysicsInertia<N, LV, AV, M> for T { }
