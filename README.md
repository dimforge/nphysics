nphysics
========

The most distinctive feature is its genericity wrt the simulation
dimension. That means you can use it for both 2-dimensional physics and
3-dimensional physics. Higher dimensions should be possible but never tested
(it you happen to have a 4-dimensional renderer let me know).

## Why another physics engine?
There are a lot of physics engine out there.
However having a physics engine written in rust is much more fun than writing
bindings and has several adventages:
- it shows that rust is suitable for soft real-time applications
- it shows how well rust behaves with highly generic code
- it shows that there is no need to write one physics engine per dimension:
  genericity wrt the dimension is possible
- in a not-that-near future, C++ will die of ugglyness. Then, people will
  search for a physics engine and **nphysics** will be there, proudly
  exhibiting its _rusty_ sexyness.

## Compilation
You will need the last rust compiler from the incomming branch.
I pull the compiler and fix my code almost every days. If you encounter
problems, make sure you have the last version.

The simplest way to build **ndemo** and all its dependencies is to do a
recursive clone:


    git clone --recursive git://github.com/sebcrozet/nphysics.git
    cd nphysics
    make deps
    make

## What is missing?
**nphysics** is a very young library and needs to learn a lot of things to
become a grown up.  Most missing features are because of missing features on
**ncollide**. Features missing from **nphysics** itself include:

- island based sleeping
- kinematic bodies
- efficient signaling system
- joints
- soft-bodies
- parallel pipeline
- GPU-based pipeline
- an efficient way to handle generically continious collision detection

## Dependencies
All dependencies are automatically cloned with a recursive clone.
If you want to grab them yourself you will need:

* **ncollide**: the collision detection library (https://github.com/sebcrozet/ncollide)
* **nalgebra**: the linear algebra library (https://github.com/sebcrozet/nalgebra)
