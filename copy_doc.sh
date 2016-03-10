#! /bin/bash

nphysics_path=../nphysics

rm -rf doc

cd $nphysics_path; cargo doc --no-deps --features "3df32"
cd -
cp -r ../nphysics/target/doc .
rm -rf doc/nphysics


for ver in 2df32 2df64 3df32 3df64
do
    cd $nphysics_path; cargo doc --features $ver --no-deps
    cd -
    cp -r ${nphysics_path}/target/doc/nphysics doc/nphysics$ver
    # Fix the links.
    sed -i "s/\\.\\.\\/nphysics/\\.\\.\\/nphysics${ver}/g" doc/nphysics${ver}/{,**}/*.html
    # Fix the documentation index (see #45).
    sed -i "s/nphysics3df32/nphysics${ver}/g" doc/nphysics${ver}/{,**}/*.html
done
