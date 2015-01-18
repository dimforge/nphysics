[![Build Status](https://travis-ci.org/sebcrozet/nphysics.svg)](https://travis-ci.org/sebcrozet/nphysics)

nphysics
========
**nphysics** is a 2 and 3-dimensional physics engine for games and animations. It uses
[ncollide](http://ncollide.org) for collision detection, and
[nalgebra](http://nalgebra.org) for vector/matrix math.

Its most distinctive feature is its genericity wrt the simulation
dimension. That means you can use it for both 2-dimensional physics and
3-dimensional physics. Higher dimensions could be possible, but **nphysics**
has not been written/tested with those in thought.

Examples are available in the `examples` directory.
There is also a short (outdated) [demonstration video](http://youtu.be/CANjXZ5rocI).

An on-line version of this documentation is available [here](http://nphysics-dev.org).

## Why another physics engine?
There are a lot of physics engine out there.
However having a physics engine written in Rust is much more fun than
writing bindings and has several advantages:

- it shows that Rust is suitable for soft real-time applications
- it shows how well Rust behaves with highly generic code
- it shows that there is no need to write two separate engine for 2d and 3d:
  genericity wrt the dimension is possible (modulo low level arithmetic
  specializations for each dimension).
- in a not-that-near future, C++ will die of ugliness. Then, people will
  search for a physics engine and **nphysics** will be there, proudly
  exhibiting its _Rusty_ sexyness.

## Compilation
You will need the last nightly build of the [Rust compiler](http://www.rust-lang.org)
and the official package manager: [Cargo](https://github.com/rust-lang/cargo).

Simply add the following to your `Cargo.toml` file if you want the 3d version
of nphysics, using 32-bits foalting point numbers:

```
[dependencies.nphysics3df32]
git = "https://github.com/sebcrozet/nphysics"
```

Depending on the accuracy or the dimension that you need, `nphysics3df32` may
be replaced by:

* `nphysics2df32` − for 2d collision detection and 32 bits precision.
* `nphysics3df32` − for 3d collision detection and 32 bits precision.
* `nphysics2df64` − for 2d collision detection and 64 bits precision.
* `nphysics3df64` − for 3d collision detection and 64 bits precision.

Use `make examples` to build the demos and execute `./your_favorite_example_here --help`
to see all the cool stuffs you can do.

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

* [ncollide](http://ncollide.org): the collision detection library.
* [nalgebra](http://nalgebra.org): the linear algebra library.

The libraries needed to compile the examples are:

* [kiss3d](http://kiss3d.org): the 3d graphics engine.
* [rust-sfml](http://www.rust-sfml.org): the 2d graphics engine.
