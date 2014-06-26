tmp=_git_distcheck
kiss3d_lib_path=./lib/kiss3d/lib
glfw_lib_path=./lib/kiss3d/lib/glfw-rs/lib
gl_lib_path=./lib/kiss3d/lib/gl-rs/lib
stb_image_lib_path=./lib/kiss3d/lib/rust-stb-image/
rsfml_lib_path=./lib/rust-sfml/lib/
freetype_lib_path=./lib/kiss3d/lib/rust-freetype/
ffmpeg_lib_path=./lib/kiss3d/lib/rust-ffmpeg/lib
nphysics_lib_path=../lib
ncollide_lib_path=../ncollide/lib
nalgebra_lib_path=../ncollide/nalgebra/lib
build_cmd=rustc -L$(nphysics_lib_path)  \
				-L$(ncollide_lib_path)  \
				-L$(nalgebra_lib_path)  \
				-L$(kiss3d_lib_path)    \
				-L$(glfw_lib_path)      \
				-L$(gl_lib_path)        \
				-L$(stb_image_lib_path) \
				-L$(rsfml_lib_path)     \
				-L$(freetype_lib_path)  \
				-L$(ffmpeg_lib_path)    \
				--out-dir ../bin
build_cmd_opt=rustc -L$(nphysics_lib_path)  \
					-L$(ncollide_lib_path)  \
					-L$(nalgebra_lib_path)  \
					-L$(kiss3d_lib_path)    \
					-L$(glfw_lib_path)      \
					-L$(gl_lib_path)     	\
					-L$(stb_image_lib_path) \
					-L$(rsfml_lib_path)     \
					-L$(freetype_lib_path)  \
					-L$(ffmpeg_lib_path)    \
					-Llib			  		\
					--out-dir ../bin --opt-level 3

lib_build_opt=rustc -L$(nphysics_lib_path)  \
					-L$(ncollide_lib_path)  \
					-L$(nalgebra_lib_path)  \
					-L$(kiss3d_lib_path)    \
					-L$(glfw_lib_path)      \
					-L$(gl_lib_path)    	\
					-L$(stb_image_lib_path) \
					-L$(rsfml_lib_path)     \
					-L$(freetype_lib_path)  \
					-L$(ffmpeg_lib_path)    \
					-Llib					\
					--out-dir lib --opt-level 3

examples: examples3d examples2d

examples3d:
	mkdir -p ../bin
	$(lib_build_opt) ./graphics3d/graphics3d.rs
	#$(build_cmd_opt) ./bezier_bezier3d.rs
	$(build_cmd_opt) ./bezier_balls3d.rs
	$(build_cmd_opt) ./ragdoll3d.rs
	$(build_cmd_opt) ./boxes_vee3d.rs
	$(build_cmd_opt) ./mesh3d.rs
	$(build_cmd_opt) ./primitives3d.rs
	$(build_cmd_opt) ./balls_vee3d.rs
	$(build_cmd_opt) ./compound3d.rs
	$(build_cmd_opt) ./cross3d.rs
	$(build_cmd_opt) ./pyramid3d.rs
	$(build_cmd_opt) ./wall3d.rs

examples2d:
	$(lib_build_opt) ./graphics2d/graphics2d.rs
	$(build_cmd_opt) ./mesh2d.rs
	$(build_cmd_opt) ./boxes_vee2d.rs
	$(build_cmd_opt) ./pyramid2d.rs
	$(build_cmd_opt) ./wall2d.rs
	$(build_cmd_opt) ./cross2d.rs
	$(build_cmd_opt) ./compound2d.rs
	$(build_cmd_opt) ./balls_vee2d.rs
	$(build_cmd_opt) ./ragdoll2d.rs

bugs:
	mkdir -p ../bin
	$(lib_build_opt) ./graphics3d/graphics3d.rs
	$(build_cmd_opt) ./regression_tests/bug_long_thin_box_one_shot_manifold.rs
	$(build_cmd_opt) ./regression_tests/bug_optimization_on_patch_boundaries_does_not_converge.rs

deps: deps2d deps3d

deps2d:
	mkdir -p lib
	make rsfml -C lib/rust-sfml

deps3d:
	mkdir -p lib
	make deps -C lib/kiss3d
	make kiss3d -C lib/kiss3d

distcheck:
	rm -rf $(tmp)
	git clone --recursive . $(tmp)
	make -C $(tmp) deps
	make -C $(tmp)
	make -C $(tmp) release
	rm -rf $(tmp)

# FIXME: not sure if all those are useful.
.PHONY:doc
.PHONY:bugs
.PHONY:deps2d
.PHONY:deps3d
.PHONY:examples2d
.PHONY:examples3d
.PHONY:examples
