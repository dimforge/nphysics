tmp=_git_distcheck
rust_sfml_rc=./lib/rust-sfml/src/rsfml/lib.rs
kiss3d_lib_path=./lib/kiss3d/lib
glfw_lib_path=./lib/kiss3d/lib/glfw-rs
gl_lib_path=./lib/kiss3d/lib/gl-rs
stb_image_lib_path=./lib/kiss3d/lib/rust-stb-image/
nphysics_lib_path=../lib
ncollide_lib_path=../ncollide/lib
nalgebra_lib_path=../ncollide/nalgebra/lib
build_cmd=rust build -L$(nphysics_lib_path)  \
					 -L$(ncollide_lib_path)  \
					 -L$(nalgebra_lib_path)  \
					 -L$(kiss3d_lib_path)    \
					 -L$(glfw_lib_path)      \
					 -L$(gl_lib_path)        \
					 -L$(stb_image_lib_path) \
					 --out-dir ../bin
build_cmd_opt=rust build -L$(nphysics_lib_path)   \
						 -L$(ncollide_lib_path)   \
						 -L$(nalgebra_lib_path)   \
						 -L$(kiss3d_lib_path)     \
						 -L$(glfw_lib_path)       \
						 -L$(gl_lib_path)     	  \
						 -L$(stb_image_lib_path)  \
						 -Llib			  		  \
						 --out-dir ../bin --opt-level 3

lib_build_opt=rust build -L$(nphysics_lib_path)  \
						 -L$(ncollide_lib_path)  \
						 -L$(nalgebra_lib_path)  \
						 -L$(kiss3d_lib_path)    \
						 -L$(glfw_lib_path)      \
						 -L$(gl_lib_path)    	 \
						 -L$(stb_image_lib_path) \
						 -Llib					 \
						 --out-dir lib --opt-level 3

examples:
	mkdir -p ../bin
	$(lib_build_opt) ./graphics3d/graphics3d.rs
	$(build_cmd_opt) ./boxes_vee3d.rs
	$(build_cmd_opt) ./cross3d.rs
	$(build_cmd_opt) ./balls_vee3d.rs
	$(build_cmd_opt) ./pyramid3d.rs
	$(build_cmd_opt) ./primitives3d.rs
	$(build_cmd_opt) ./wall3d.rs
	$(build_cmd_opt) ./compound3d.rs
	$(lib_build_opt) ./graphics2d/graphics2d.rs
	$(build_cmd_opt) ./boxes_vee2d.rs
	$(build_cmd_opt) ./cross2d.rs
	$(build_cmd_opt) ./pyramid2d.rs
	$(build_cmd_opt) ./compound2d.rs
	$(build_cmd_opt) ./balls_vee2d.rs
	$(build_cmd_opt) ./wall2d.rs

deps:
	mkdir -p lib
	rust build $(rust_sfml_rc) --opt-level 3 --out-dir lib
	make deps -C ..
	make deps -C lib/kiss3d
	make -C lib/kiss3d
	make -C ..

distcheck:
	rm -rf $(tmp)
	git clone --recursive . $(tmp)
	make -C $(tmp) deps
	make -C $(tmp)
	make -C $(tmp) release
	rm -rf $(tmp)

.PHONY:doc

