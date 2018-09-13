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

## Compilation
You will need the latest release of the [Rust compiler](http://www.rust-lang.org)
and the official package manager: [Cargo](https://github.com/rust-lang/cargo).

If you want to use the 2D version of `nphysics`, add the crate named
`nphysics2d` to your dependencies:

```ignore
[dependencies]
nphysics2d = "0.8"
```

For the 3D version, add the crate named `nphysics3d`:

```ignore
[dependencies]
nphysics3d = "0.8"
```

Use `make examples` to build the demos and execute `./your_favorite_example_here --help`
to see all the cool stuffs you can do.

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

## What is missing?
**nphysics** is a very young library and needs to learn a lot of things to
become a grown up. Many missing features are because of missing features on
**ncollide**. Features missing from **nphysics** itself include:

- kinematic bodies
- efficient signaling system
- more joints, joint limits, joint motors and breakable joints.
- soft-bodies (see https://github.com/natal/roft for a draft)
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
#![deny(missing_docs)]
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

/*
 * The two following crates are pulled-in for
 * measuring time.
 */
#[cfg(not(any(target_arch = "wasm32", target_arch = "asmjs")))]
extern crate time;

#[cfg(all(
  any(target_arch = "wasm32", target_arch = "asmjs"),
  feature = "stdweb",
))]
#[macro_use]
extern crate stdweb;

#[cfg(all(
  any(target_arch = "wasm32", target_arch = "asmjs"),
  feature = "use-wasm-bindgen",
))]
extern crate wasm_bindgen;

//#[cfg(test)]
//extern crate test;

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
  use algebra::{Force3, Inertia3, Velocity3};
  use na::{
    Dynamic, Isometry3, Matrix3, Matrix6, MatrixMN, MatrixSlice6xX, MatrixSliceMut6xX, Point3,
    Translation3, UnitQuaternion, Vector3, Vector6, U3, U6,
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

  /// The rotation matrix type.
  pub type Rotation<N> = UnitQuaternion<N>;

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
  use algebra::{Force2, Inertia2, Velocity2};
  use na::{
    Dynamic, Isometry2, Matrix1, Matrix3, MatrixMN, MatrixSlice3xX, MatrixSliceMut3xX, Point2,
    Translation2, UnitComplex, Vector1, Vector2, Vector3, U1, U2, U3,
  };

  /// The maximum number of possible rotations and translations of a rigid body.
  pub const SPATIAL_DIM: usize = 3;
  /// The maximum number of possible rotations of a rigid body.
  pub const ANGULAR_DIM: usize = 1;
  /// The maximum number of possible translations of a rigid body.
  pub const DIM: usize = 2;

  /// The dimension of the ambiant space.
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

  /// The rotation matrix type.
  pub type Rotation<N> = UnitComplex<N>;

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

  /// Square matrix with dimension `SpatialDim × SpatialDim`.
  pub type SpatialMatrix<N> = Matrix3<N>;

  /// The type of a constraint jacobian in twist coordinates.
  pub type Jacobian<N> = MatrixMN<N, U3, Dynamic>;

  /// The type of a slice of the constraint jacobian in twist coordinates.
  pub type JacobianSlice<'a, N> = MatrixSlice3xX<'a, N>;

  /// The type of a mutable slice of the constraint jacobian in twist coordinates.
  pub type JacobianSliceMut<'a, N> = MatrixSliceMut3xX<'a, N>;
}
