tmp=_git_distcheck

all:
	cd build/nphysics2df32; cargo build --release
	cd build/nphysics2df64; cargo build --release
	cd build/nphysics3df32; cargo build --release
	cd build/nphysics3df64; cargo build --release

examples:
	cd examples3d; cargo build --release
	cd examples2d; cargo build --release

distcheck:
	rm -rf $(tmp)
	git clone . $(tmp)
	make -C $(tmp)
	make -C $(tmp) examples2d
	make -C $(tmp) examples3d
	rm -rf $(tmp)

doc:
	cargo doc

clean:
	cargo clean

.PHONY:doc
# FIXME: uggly!
.PHONY:examples
.PHONY:bugs
