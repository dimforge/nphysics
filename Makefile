tmp=_git_distcheck
nphysics_doc_path=doc
nalgebra_lib_path=./ncollide/nalgebra/lib
ncollide_lib_path=./ncollide/lib

all:
	cargo build -u --release

bugs:
	make -C examples bugs

examples:
	cd examples; cargo build -u --release

distcheck:
	rm -rf $(tmp)
	git clone . $(tmp)
	make -C $(tmp)
	make -C $(tmp) examples
	rm -rf $(tmp)

doc:
	mkdir -p $(nphysics_doc_path)
	rustdoc src/lib3df32.rs -L$(nalgebra_lib_path) -L$(ncollide_lib_path) --cfg dim3 --cfg f32
	rustdoc src/lib3df64.rs -L$(nalgebra_lib_path) -L$(ncollide_lib_path) --cfg dim3 --cfg f64
	rustdoc src/lib2df32.rs -L$(nalgebra_lib_path) -L$(ncollide_lib_path) --cfg dim2 --cfg f32
	rustdoc src/lib2df64.rs -L$(nalgebra_lib_path) -L$(ncollide_lib_path) --cfg dim2 --cfg f64

clean:
	cargo clean

.PHONY:doc
# FIXME: uggly!
.PHONY:examples
.PHONY:bugs
