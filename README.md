<p align="center">
  <img src="http://nphysics.org/img/logo_nphysics_full.svg" alt="crates.io">
</p>
<p align="center">
    <a href="https://crates.io/crates/nphysics">
         <img src="http://meritbadge.herokuapp.com/nphysics3d?style=flat-square" alt="crates.io">
    </a>
    <a href="https://travis-ci.org/rustsim/nphysics">
        <img src="https://travis-ci.org/rustsim/nphysics.svg?branch=master" alt="Build status">
    </a>
</p>
<p align = "center">
    <strong>
        <a href="http://nphysics.org">Users guide</a> | <a href="http://nphysics.org/rustdoc/nphysics2d/index.html">2D Documentation</a> | <a href="http://nphysics.org/rustdoc/nphysics3d/index.html">3D Documentation</a> | <a href="https://discourse.nphysics.org">Forum</a>
    </strong>
</p>

-----

<p align = "center">
  <i>Click one of those buttons if you wish to donate to support the development of</i> <b>nphysics</b>:
</p>

<p align = "center">
<a href="https://www.patreon.com/bePatron?u=7111380" ><img src="https://c5.patreon.com/external/logo/become_a_patron_button.png" alt="Become a Patron!" /></a>
&nbsp;&nbsp;&nbsp;&nbsp;
<a href="https://liberapay.com/sebcrozet/donate"><img alt="Donate using Liberapay" src="https://liberapay.com/assets/widgets/donate.svg"></a>
</p>

nphysics
========
**nphysics** is a 2 and 3-dimensional physics engine for games and animations.
It uses [ncollide](http://ncollide.org) for collision detection, and
[nalgebra](http://nalgebra.org) for vector/matrix math. 2D and 3D
implementations both share the same code!


Examples are available in the `examples2d` and `examples3d` directories. Interactive
3D are available [there](http://demo.nphysics.org/). Because those demos are based on
WASM and WebGl 1.0 they should work on most modern browsers. Feel free to ask for help
and discuss features on the official [user forum](http://users.nphysics.org).

## Why another physics engine?
There are a lot of physics engine out there.
However having a physics engine written in Rust is much more fun than
writing bindings and has several advantages:

- It shows that Rust is suitable for soft real-time applications.
− It features an efficient implementation of multibodies using the reduced-coordinates approach. Constraint-based joints are also supported.
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
nphysics2d = "0.9"
```

For the 3D version, add the crate named `nphysics3d`:

```ignore
[dependencies]
nphysics3d = "0.9"
```

## Features
- Static, dynamic, and kinematic rigid bodies.
- Common convex primitives: box, ball, convex polyhedron.
- Concave geometries built from convex primitives: compound geometries, trinangle mesh, polylines.
- Multibodies using reduced-coordinates approaches or constraints-based joints.
- Multibody joint limits and motors.
- Stable stacking due to non-linear a position-based penetration correction and one-shot contact manifold generation.
- Island based sleeping (objects deactivation when they are at rest).
- Ray casting.
- Sensors.
- WASM support.

## What is missing?
**nphysics** is a very young library and needs to learn a lot of things to
become a grown up. Many missing features are because of missing features on
**ncollide**. Features missing from **nphysics** itself include:

- Breakable joints.
- Soft-bodies.
- Fluids.
- Parallel pipeline.
- GPU-based pipeline.

## Dependencies
The libraries needed to compile the physics engine are:

* [ncollide](http://ncollide.org): the collision detection library.
* [nalgebra](http://nalgebra.org): the linear algebra library.

The libraries needed to compile the examples are:

* [kiss3d](http://kiss3d.org): the 3d graphics engine.
* [rust-sfml](http://www.rust-sfml.org): the 2D graphics engine. See [windows installation instructions](https://github.com/jeremyletang/rust-sfml/wiki/How-to-use-rust-sfml-on-Windows).
