/*!
nphysics
========
**nphysics** is a 2 and 3-dimensional physics engine for games and animations.
It uses [ncollide](http://ncollide.org) for collision detection, and
[nalgebra](http://nalgebra.org) for vector/matrix math. 2D and 3D
implementations both share the same code!


Examples are available in the `examples2d` and `examples3d` directories. There
is also a short (outdated) [demonstration video](http://youtu.be/CANjXZ5rocI).
An on-line version of this documentation is available
[here](http://nphysics.org). Feel free to ask for help and discuss features on
the official [user forum](http://users.nphysics.org).

## Why another physics engine?
There are a lot of physics engine out there.
However having a physics engine written in Rust is much more fun than
writing bindings and has several advantages:

- it shows that Rust is suitable for soft real-time applications
- it shows how well Rust behaves with highly generic code
- it shows that there is no need to write two separate engine for 2D and 3D:
  genericity wrt the dimension is possible (modulo low level arithmetic
  specializations for each dimension).
- in a not-that-near future, C++ will die of ugliness. Then, people will
  search for a physics engine and **nphysics** will be there, proudly
  exhibiting its _Rusty_ sexyness.


## Features
- Static and dynamic rigid bodies.
- Common convex primitives: cone, box, ball, cylinder.
- Concave geometries build from convex primitives (aka. compound geometries).
- Stable stacking.
- Island based sleeping (objects deactivation).
- Ray casting.
- Swept sphere based continuous collision detection.
- Ball-in-socket joint.
- FixedJoint joint.
- Sensors.
- Deformable bodies (aka. soft-bodies)
- Kinematic bodies

## What is missing?
**nphysics** is a very young library and needs to learn a lot of things to
become a grown up. Many missing features are because of missing features on
**ncollide**. Features missing from **nphysics** itself include:

- more joints, joint limits, joint motors and breakable joints.
- parallel pipeline
- GPU-based pipeline

## Dependencies
All dependencies are automatically cloned with a recursive clone.
The libraries needed to compile the physics engine are:

* [ncollide](http://ncollide.org): the collision detection library.
* [nalgebra](http://nalgebra.org): the linear algebra library.

The libraries needed to compile the examples are:

*/

#![deny(non_camel_case_types)]
#![deny(unused_parens)]
#![deny(non_upper_case_globals)]
#![deny(unused_qualifications)]
#![warn(missing_docs)] // XXX: deny that
#![deny(unused_results)]
#![warn(non_camel_case_types)]
#![allow(missing_copy_implementations)]
#![doc(html_root_url = "http://nphysics-dev.org/doc")]

#[macro_use]
extern crate approx;
#[macro_use]
extern crate downcast;

extern crate alga;
extern crate nalgebra as na;
#[cfg(feature = "dim2")]
extern crate ncollide2d as ncollide;
#[cfg(feature = "dim3")]
extern crate ncollide3d as ncollide;
extern crate num_traits as num;
extern crate slab;
extern crate either;

/*
 * The two following crates are pulled-in for
 * measuring time.
 */
#[cfg(not(any(target_arch = "wasm32", target_arch = "asmjs")))]
extern crate time;

#[cfg(any(target_arch = "wasm32", target_arch = "asmjs"))]
#[macro_use]
extern crate stdweb;

//#[cfg(test)]
//extern crate test;

macro_rules! try_ret {
    ($val: expr) => {
        try_ret!($val, ())
    };
    ($val: expr, $ret: expr) => {
        if let Some(val) = $val {
            val
        } else {
            return $ret;
        }
    }
}

macro_rules! try_continue {
    ($val: expr) => {
        if let Some(val) = $val {
            val
        } else {
            continue;
        }
    }
}

macro_rules! desc_setters(
    ($($with_method: ident, $set_method: ident $(, $arg: ident: $t: ty)*)*) => {
        $(
            #[inline]
            pub fn $with_method(mut self $(, $arg: $t)*) -> Self {
                $(
                    self.$arg = $arg;
                )*
                self
            }

            #[inline]
            pub fn $set_method(&mut self $(, $arg: $t)*) -> &mut Self {
                $(
                    self.$arg = $arg;
                )*
                self
            }
        )*
    }
);

macro_rules! desc_custom_setters(
    ($($this: ident.$with_method: ident, $set_method: ident $(, $arg: ident: $t: ty)* | $imp: expr)*) => {
        $(
            #[inline]
            pub fn $with_method(mut $this $(, $arg: $t)*) -> Self {
                $imp
                $this
            }

            #[inline]
            pub fn $set_method(&mut $this $(, $arg: $t)*) -> &mut Self {
                $imp
                $this
            }
        )*
    }
);

macro_rules! desc_custom_getters(
    ($($this: ident.$get_method: ident: $t: ty | $imp: expr)*) => {
        $(
            #[inline]
            pub fn $get_method(&$this) -> $t {
                $imp
            }
        )*
    }
);

macro_rules! desc_getters(
    ($([val] $val: ident: $t: ty)* $([ref] $ref_val: ident: $ref_t: ty)*) => {
        $(
            #[inline]
            pub fn $val(&self) -> $t {
                self.$val
            }
        )*
        $(
            #[inline]
            pub fn $ref_val(&self) -> &$ref_t {
                &self.$ref_val
            }
        )*
    }
);

macro_rules! user_data_accessors(
    () => {
        #[inline]
        pub fn user_data(&self) -> Option<&(Any + Send + Sync)> {
            self.user_data.as_ref().map(|d| &**d)
        }

        #[inline]
        pub fn user_data_mut(&mut self) -> Option<&mut (Any + Send + Sync)> {
            self.user_data.as_mut().map(|d| &mut **d)
        }

        #[inline]
        pub fn set_user_data(&mut self, data: Option<Box<Any + Send + Sync>>) -> Option<Box<Any + Send + Sync>> {
            std::mem::replace(&mut self.user_data, data)
        }

        #[inline]
        pub fn take_user_data(&mut self) -> Option<Box<Any + Send + Sync>> {
            self.user_data.take()
        }
    }
);

pub mod algebra;
pub mod counters;
pub mod detection;
pub mod force_generator;
pub mod joint;
pub mod object;
pub mod solver;
pub mod utils;
pub mod volumetric;
pub mod world;
// mod tests;

/// Compilation flags dependent aliases for mathematical types.
#[cfg(feature = "dim3")]
pub mod math {
    use crate::algebra::{Force3, Inertia3, Velocity3};
    use na::{
        Dynamic, Isometry3, Matrix3, Matrix6, MatrixMN, MatrixSlice6xX, MatrixSliceMut6xX, Point3,
        Translation3, U3, U6, UnitQuaternion, Vector3, Vector6, Rotation3
    };

    /// The maximum number of possible rotations and translations of a rigid body.
    pub const SPATIAL_DIM: usize = 6;
    /// The maximum number of possible rotations of a rigid body.
    pub const ANGULAR_DIM: usize = 3;
    /// The maximum number of possible translations of a rigid body.
    pub const DIM: usize = 3;

    /// The dimension of the ambiant space.
    pub type Dim = U3;

    /// The dimension of a spatial vector.
    pub type SpatialDim = U6;

    /// The dimension of the rotations.
    pub type AngularDim = U3;

    /// The point type.
    pub type Point<N> = Point3<N>;

    /// The angular vector type.
    pub type AngularVector<N> = Vector3<N>;

    /// The vector type.
    pub type Vector<N> = Vector3<N>;

    /// The vector type with dimension `SpatialDim × 1`.
    pub type SpatialVector<N> = Vector6<N>;

    /// The orientation type.
    pub type Orientation<N> = Vector3<N>;

    /// The transformation matrix type.
    pub type Isometry<N> = Isometry3<N>;

    /// The rotation type.
    pub type Rotation<N> = UnitQuaternion<N>;

    /// The rotation matrix type.
    pub type RotationMatrix<N> = Rotation3<N>;

    /// The translation type.
    pub type Translation<N> = Translation3<N>;

    /// The velocity type combining the linear velocity and the angular velocity.
    pub type Velocity<N> = Velocity3<N>;

    /// The force type combining a linear force and a torque.
    pub type Force<N> = Force3<N>;

    /// The inertia tensor type.
    pub type AngularInertia<N> = Matrix3<N>;

    /// The inertia type.
    pub type Inertia<N> = Inertia3<N>;

    /// The inertia matrix type.
    pub type InertiaMatrix<N> = Matrix6<N>;

    /// Square matrix with dimension `Dim × Dim`.
    pub type Matrix<N> = Matrix3<N>;

    /// Square matrix with dimension `SpatialDim × SpatialDim`.
    pub type SpatialMatrix<N> = Matrix6<N>;

    /// The type of a constraint jacobian in twist coordinates.
    pub type Jacobian<N> = MatrixMN<N, U6, Dynamic>;

    /// The type of a slice of the constraint jacobian in twist coordinates.
    pub type JacobianSlice<'a, N> = MatrixSlice6xX<'a, N>;

    /// The type of a mutable slice of the constraint jacobian in twist coordinates.
    pub type JacobianSliceMut<'a, N> = MatrixSliceMut6xX<'a, N>;
}

/// Compilation flags dependent aliases for mathematical types.
#[cfg(feature = "dim2")]
pub mod math {
    use crate::algebra::{Force2, Inertia2, Velocity2};
    use na::{
        Dynamic, Isometry2, Matrix1, Matrix3, MatrixMN, MatrixSlice3xX, MatrixSliceMut3xX, Point2,
        Translation2, U1, U2, U3, UnitComplex, Vector1, Vector2, Vector3, Rotation2, Matrix2
    };

    /// The maximum number of possible rotations and translations of a rigid body.
    pub const SPATIAL_DIM: usize = 3;
    /// The maximum number of possible rotations of a rigid body.
    pub const ANGULAR_DIM: usize = 1;
    /// The maximum number of possible translations of a rigid body.
    pub const DIM: usize = 2;

    /// The dimension of the ambient space.
    pub type Dim = U2;

    /// The dimension of the rotation.
    pub type AngularDim = U1;

    /// The dimension of a spatial vector.
    pub type SpatialDim = U3;

    /// The point type.
    pub type Point<N> = Point2<N>;

    /// The vector type with dimension `SpatialDim × 1`.
    pub type SpatialVector<N> = Vector3<N>;

    /// The angular vector type.
    pub type AngularVector<N> = Vector1<N>;

    /// The vector type.
    pub type Vector<N> = Vector2<N>;

    /// The orientation type.
    pub type Orientation<N> = Vector1<N>;

    /// The transformation matrix type.
    pub type Isometry<N> = Isometry2<N>;

    /// The rotation type.
    pub type Rotation<N> = UnitComplex<N>;

    /// The rotation matrix type.
    pub type RotationMatrix<N> = Rotation2<N>;

    /// The translation type.
    pub type Translation<N> = Translation2<N>;

    /// The velocity type combining the linear velocity and the angular velocity.
    pub type Velocity<N> = Velocity2<N>;

    /// The force type combining a linear force and a torque.
    pub type Force<N> = Force2<N>;

    /// The inertia tensor type.
    pub type AngularInertia<N> = Matrix1<N>;

    /// The inertia type.
    pub type Inertia<N> = Inertia2<N>;

    /// The inertia matrix type.
    pub type InertiaMatrix<N> = Matrix3<N>;

    /// Square matrix with dimension `Dim × Dim`.
    pub type Matrix<N> = Matrix2<N>;

    /// Square matrix with dimension `SpatialDim × SpatialDim`.
    pub type SpatialMatrix<N> = Matrix3<N>;

    /// The type of a constraint jacobian in twist coordinates.
    pub type Jacobian<N> = MatrixMN<N, U3, Dynamic>;

    /// The type of a slice of the constraint jacobian in twist coordinates.
    pub type JacobianSlice<'a, N> = MatrixSlice3xX<'a, N>;

    /// The type of a mutable slice of the constraint jacobian in twist coordinates.
    pub type JacobianSliceMut<'a, N> = MatrixSliceMut3xX<'a, N>;
}
