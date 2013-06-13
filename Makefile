nalgebra_lib_path=./ncollide/nalgebra/lib
ncollide_lib_path=./ncollide/lib

all:
	rust build -L$(nalgebra_lib_path) -L$(ncollide_lib_path) src/nphysics.rc --out-dir lib --opt-level 3

deps:
	make deps -C ncollide
	make -C ncollide

doc:
	rust doc src/nphysics.rc --output-dir doc

.PHONY:doc
