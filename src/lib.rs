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
nphysics2d = "0.6"
```

For the 3D version, add the crate named `nphysics3d`:

```ignore
[dependencies]
nphysics3d = "0.6"
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
- Fixed joint.
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
#![warn(missing_docs)]
#![deny(unused_results)]
#![warn(non_camel_case_types)]
#![allow(missing_copy_implementations)]
#![doc(html_root_url = "http://nphysics-dev.org/doc")]

extern crate num_traits as num;
extern crate rustc_serialize;
#[cfg(test)]
#[macro_use]
extern crate approx;
extern crate alga;
extern crate nalgebra as na;
extern crate ncollide;
//#[cfg(test)]
//extern crate test;

pub mod aliases;

pub mod integration;
pub mod detection;
pub mod resolution;
pub mod world;
pub mod object;
pub mod utils;
pub mod volumetric;
// mod tests;

mod rc;

pub use self::rc::{Rc, Ref, RefMut};

/// Compilation flags dependent aliases for mathematical types.
#[cfg(feature = "dim3")]
pub mod math {
    use na::{Point3, Vector3, Matrix3, UnitQuaternion, Translation3, Isometry3};

    /// The point type.
    pub type Point<N> = Point3<N>;

    /// The vector type.
    pub type Vector<N> = Vector3<N>;

    /// The orientation type.
    pub type Orientation<N> = Vector3<N>;

    /// The transformation matrix type.
    pub type Isometry<N> = Isometry3<N>;

    /// The rotation matrix type.
    pub type Rotation<N> = UnitQuaternion<N>;

    /// The translation type.
    pub type Translation<N> = Translation3<N>;

    /// The inertia tensor type.
    pub type AngularInertia<N> = Matrix3<N>;
}

/// Compilation flags dependent aliases for mathematical types.
#[cfg(feature = "dim2")]
pub mod math {
    use na::{Point2, Vector1, Vector2, Matrix1, UnitComplex, Translation2, Isometry2};

    /// The point type.
    pub type Point<N> = Point2<N>;

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

    /// The inertia tensor type.
    pub type AngularInertia<N> = Matrix1<N>;
}
