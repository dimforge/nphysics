#! /bin/bash

nphysics_path=../nphysics

rm -rf doc

cd $nphysics_path; make doc
cd -
cp -r $nphysics_path/build/nphysics2d/target/doc doc
cp -r $nphysics_path/build/nphysics3d/target/doc/nphysics3d doc
cp -r $nphysics_path/build/nphysics3d/target/doc/src/nphysics3d doc/src
