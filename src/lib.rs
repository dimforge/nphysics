/*!
nphysics
========
**nphysics** is a 2 and 3-dimensional physics engine for games and animations. It uses
[ncollide](http://ncollide.org) for collision detection, and
[nalgebra](http://nalgebra.org) for vector/matrix math.

Its most distinctive feature is its genericity wrt the simulation
dimension. That means you can use it for both 2-dimensional physics and
3-dimensional physics. Higher dimensions could be possible, but **nphysics**
has not be written/tested with those in thought.

Examples are available on the `examples` directory.
There is also a short (outdated) [demonstration video](http://youtu.be/CANjXZ5rocI).

## Why another physics engine?
There are a lot of physics engine out there.
However having a physics engine written in rust is much more fun than
writing bindings and has several advantages:

- it shows that rust is suitable for soft real-time applications
- it shows how well rust behaves with highly generic code
- it shows that there is no need to write two separate engine for 2d and 3d:
  genericity wrt the dimension is possible (modulo low level arithmetic
  specializations for each dimension).
- in a not-that-near future, C++ will die of ugliness. Then, people will
  search for a physics engine and **nphysics** will be there, proudly
  exhibiting its _rusty_ sexyness.

## Compilation
You will need the last nightly build of the [rust compiler](http://www.rust-lang.org)
and the official package manager: [cargo](https://github.com/rust-lang/cargo).

Simply add the following to your `Cargo.toml` file:

```ignore
[dependencies.nphysics3df32]
git = "https://github.com/sebcrozet/nphysics"
```

## Features
- static and dynamic rigid bodies
- common convex primitives: cone, box, ball, cylinder
- concave geometries build from convex primitives (aka. compound geometries)
- stable stacking
- island based sleeping (objects deactivation)
- ray casting
- swept sphere based continuous collision detection
- ball-in-socket joint
- fixed joint

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

* [ncollide](https://github.com/sebcrozet/ncollide): the collision detection library.
* [nalgebra](https://github.com/sebcrozet/nalgebra): the linear algebra library.

The libraries needed to compile the examples are:

* [kiss3d](https://github.com/sebcrozet/kiss3d): the 3d graphics engine.
* [rust-sfml](https://github.com/JeremyLetang/rust-sfml): the 2d graphics engine.
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

extern crate num;
extern crate rustc_serialize;
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
mod tests;


/// Compilation flags dependent aliases for mathematical types.
#[cfg(feature = "3d")]
pub mod math {
    use na::{Pnt3, Vec3, Mat3, Rot3, Iso3};

    /// The scalar type.
    #[cfg(feature = "f32")]
    pub type Scalar = f32;

    /// The scalar type.
    #[cfg(feature = "f64")]
    pub type Scalar = f64;

    /// The point type.
    pub type Point = Pnt3<Scalar>;

    /// The vector type.
    pub type Vect = Vec3<Scalar>;

    /// The orientation type.
    pub type Orientation = Vec3<Scalar>;

    /// The transformation matrix type.
    pub type Matrix = Iso3<Scalar>;

    /// The rotation matrix type.
    pub type RotationMatrix = Rot3<Scalar>;

    /// The inertia tensor type.
    pub type AngularInertia = Mat3<Scalar>;
}

/// Compilation flags dependent aliases for mathematical types.
#[cfg(feature = "2d")]
pub mod math {
    use na::{Pnt2, Vec1, Vec2, Mat1, Rot2, Iso2};

    /// The scalar type.
    #[cfg(feature = "f32")]
    pub type Scalar = f32;

    /// The scalar type.
    #[cfg(feature = "f64")]
    pub type Scalar = f64;

    /// The point type.
    pub type Point = Pnt2<Scalar>;

    /// The vector type.
    pub type Vect = Vec2<Scalar>;

    /// The orientation type.
    pub type Orientation = Vec1<Scalar>;

    /// The transformation matrix type.
    pub type Matrix = Iso2<Scalar>;

    /// The rotation matrix type.
    pub type RotationMatrix = Rot2<Scalar>;

    /// The inertia tensor type.
    pub type AngularInertia = Mat1<Scalar>;
}

/// Compilation flags dependent aliases for mathematical types.
#[cfg(feature = "4d")]
pub mod math {
    use na::{Pnt4, Vec4, Mat4, Rot4, Iso4};

    /// The scalar type.
    #[cfg(feature = "f32")]
    pub type Scalar = f32;

    /// The scalar type.
    #[cfg(feature = "f64")]
    pub type Scalar = f64;

    /// The point type.
    pub type Point = Pnt4<Scalar>;

    /// The vector type.
    pub type Vect = Vec4<Scalar>;

    /// The orientation type.
    pub type Orientation = Vec4<Scalar>;

    /// The transformation matrix type.
    pub type Matrix = Iso4<Scalar>;

    /// The rotation matrix type.
    pub type RotationMatrix = Rot4<Scalar>;

    /// The inertia tensor type.
    pub type AngularInertia = Mat4<Scalar>;
}
