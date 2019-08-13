#! /bin/bash

tmp=$(mktemp -d)

echo "$tmp"

cp -r src_testbed "$tmp"/.
cp -r LICENSE README.md "$tmp"/.

#### Publish the 2D version.
#sed 's#\.\./\.\./src#src#g' build/nphysics2d/Cargo.toml > "$tmp"/Cargo.toml
#currdir=$(pwd)
#cd "$tmp" && cargo publish
#cd "$currdir" || exit
#
#
#### Publish the 3D version.
#sed 's#\.\./\.\./src#src#g' build/nphysics3d/Cargo.toml > "$tmp"/Cargo.toml
#cp -r LICENSE README.md "$tmp"/.
#cd "$tmp" && cargo publish
#
#rm -rf "$tmp"

### Publish the 2D version.
sed 's#\.\./\.\./src#src#g' build/nphysics_testbed2d/Cargo.toml > "$tmp"/Cargo.toml
currdir=$(pwd)
cd "$tmp" && cargo publish
cd "$currdir" || exit


### Publish the 3D version.
sed 's#\.\./\.\./src#src#g' build/nphysics_testbed3d/Cargo.toml > "$tmp"/Cargo.toml
cp -r LICENSE README.md "$tmp"/.
cd "$tmp" && cargo publish

rm -rf "$tmp"
