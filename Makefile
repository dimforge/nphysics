tmp=_git_distcheck
nalgebra_lib_path=./ncollide/nalgebra/lib
ncollide_lib_path=./ncollide/lib

all:
	mkdir -p lib
	rust build -L$(nalgebra_lib_path) -L$(ncollide_lib_path) src/nphysics.rc --out-dir lib --opt-level 3

deps:
	make deps -C ncollide
	make -C ncollide

bench:
	mkdir -p $(ncollide_lib_path)
	rustc -L$(nalgebra_lib_path) --test -L$(ncollide_lib_path) src/nphysics.rc --opt-level 3 -o bench~ && ./bench~ --bench

distcheck:
	rm -rf $(tmp)
	git clone --recursive . $(tmp)
	make -C $(tmp) deps
	make -C $(tmp)
	rm -rf $(tmp)

doc:
	rust doc src/nphysics.rc --output-dir doc

.PHONY:doc
