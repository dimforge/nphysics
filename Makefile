tmp=_git_distcheck
nalgebra_lib_path=./ncollide/nalgebra/lib
ncollide_lib_path=./ncollide/lib

#
# To build examples
#
rust_sfml_rc=lib/rust-sfml/src/rsfml.rc
kiss3d_lib_path=lib/kiss3d/lib
glfw_lib_path=lib/kiss3d/lib/glfw-rs/lib
gl_lib_path=lib/kiss3d/lib/gl-rs
stb_image_lib_path=lib/kiss3d/lib/rust-stb-image/
build_cmd_opt=rust build -L$(ncollide_lib_path)   \
						 -L$(nalgebra_lib_path)   \
						 -L$(kiss3d_lib_path)     \
						 -L$(glfw_lib_path)       \
						 -L$(gl_lib_path)     	  \
						 -L$(stb_image_lib_path)  \
						 --out-dir bin --opt-level 3
lib_build_opt=rust build -L$(nphysics_lib_path)  \
						 -L$(ncollide_lib_path)  \
						 -L$(nalgebra_lib_path)  \
						 -L$(kiss3d_lib_path)    \
						 -L$(glfw_lib_path)      \
						 -L$(gl_lib_path)    	 \
						 -L$(stb_image_lib_path) \
						 --out-dir lib --opt-level 3

all:
	mkdir -p lib
	rust build -L$(nalgebra_lib_path) -L$(ncollide_lib_path) src/nphysics.rc --out-dir lib --opt-level 3

deps:
	make deps -C ncollide
	make -C ncollide

bench:
	mkdir -p $(ncollide_lib_path)
	rustc -L$(nalgebra_lib_path) --test -L$(ncollide_lib_path) src/nphysics.rc --opt-level 3 -o bench~ && ./bench~ --bench

test:
	mkdir -p bin
	make -C examples

# examples dependencies
examples_deps:
	make -C examples deps

examples:
	mkdir -p bin
	make -C examples

distcheck:
	rm -rf $(tmp)
	git clone --recursive . $(tmp)
	make -C $(tmp) deps
	make -C $(tmp)
	make examples_deps -C $(tmp)
	make examples -C $(tmp)
	rm -rf $(tmp)

doc:
	rust doc src/nphysics.rc --output-dir doc

.PHONY:doc
# FIXME: uggly!
.PHONY:examples
