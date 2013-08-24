nphysics
========
**nphysics** is a 2 and 3-dimensional multiphysics engine for games and
animations.

Its most distinctive feature is its genericity wrt the simulation
dimension. That means you can use it for both 2-dimensional multiphysics and
3-dimensional multiphysics. Hight dimensions could be possible, but **nphysics**
has not be written/tested with those in thought.

See the [ndemo](https://github.com/sebcrozet/ndemo) project for demonstrations.
There is also a short [demonstration video](http://youtu.be/CANjXZ5rocI).

## Why another multiphysics engine?
There are a lot of multiphysics engine out there.
However having a multiphysics engine written in rust is much more fun than
writing bindings and has several advantages:
- it shows that rust is suitable for soft real-time applications
- it shows how well rust behaves with highly generic code
- it shows that there is no need to write two separate engine for 2d and 3d:
  genericity wrt the dimension is possible (modulo low level arithmetic
  specializations for each dimension).
- in a not-that-near future, C++ will die of ugliness. Then, people will
  search for a multiphysics engine and **nphysics** will be there, proudly
  exhibiting its _rusty_ sexyness.

## Compilation
You will need the last rust compiler from the master branch.
If you encounter problems, make sure you have the last version before creating an issue.

The simplest way to build **nphysics** and all its dependencies is to do a
recursive clone:


    git clone --recursive git://github.com/sebcrozet/nphysics.git
    cd nphysics
    make deps
    make

## What is missing?
**nphysics** is a very young library and needs to learn a lot of things to
become a grown up. Many missing features are because of missing features on
**ncollide**. Features missing from **nphysics** itself include:

- kinematic bodies
- efficient signaling system
- joints
- soft-bodies (see https://github.com/natal/roft for a draft)
- parallel pipeline
- GPU-based pipeline

## Dependencies
All dependencies are automatically cloned with a recursive clone.
If you want to grab them yourself you will need:

* [ncollide](https://github.com/sebcrozet/ncollide): the collision detection library.
* [nalgebra](https://github.com/sebcrozet/nalgebra): the linear algebra library.
