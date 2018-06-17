<p align="center">
    <a href="https://crates.io/crates/nphysics">
         <img src="http://meritbadge.herokuapp.com/nphysics3d?style=flat-square" alt="crates.io">
    </a>
    <a href="https://travis-ci.org/sebcrozet/nphysics">
        <img src="https://travis-ci.org/sebcrozet/nphysics.svg?branch=master" alt="Build status">
    </a>
</p>
<p align = "center">
    <strong>
        <a href="http://nphysics.org/doc/nphysics2d">2D Documentation</a> | <a href="http://nphysics.org/doc/nphysics3d">3D Documentation</a> | <a href="https://discourse.nphysics.org">Forum</a>
    </strong>
</p>


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

- It shows that Rust is suitable for soft real-time applications.
- It shows how well Rust behaves with highly generic code.
- It shows that there is no need to write two separate engines for 2D and 3D:
  genericity wrt the dimension is possible (modulo low level arithmetic
  specializations for each dimension).
- In a not-that-near future, C++ will die of ugliness. Then, people will
  search for a physics engine and **nphysics** will be there, proudly
  exhibiting its _Rusty_ sexiness.

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
- Concave geometries built from convex primitives (aka. compound geometries).
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

- Kinematic bodies.
- Efficient signaling system.
- More joints, joint limits, joint motors and breakable joints.
- Soft-bodies.
- Parallel pipeline.
- GPU-based pipeline.

## Dependencies
All dependencies are automatically cloned with a recursive clone.
The libraries needed to compile the physics engine are:

* [ncollide](http://ncollide.org): the collision detection library.
* [nalgebra](http://nalgebra.org): the linear algebra library.

The libraries needed to compile the examples are:

* [kiss3d](http://kiss3d.org): the 3d graphics engine.
* [rust-sfml](http://www.rust-sfml.org): the 2D graphics engine. See [windows installation instructions](https://github.com/jeremyletang/rust-sfml/wiki/How-to-use-rust-sfml-on-Windows).
