tmp=_git_distcheck

all:
	make -C build/nphysics2d
	make -C build/nphysics3d

examples:
	cd examples3d; cargo build --release
	cd examples2d; cargo build --release

doc:
	make doc -C build/nphysics2d
	make doc -C build/nphysics3d
	
test:
	make test -C build/nphysics2d
	make test -C build/nphysics3d
	

clean:
	cargo clean

.PHONY:doc
# FIXME: uggly!
.PHONY:examples
.PHONY:bugs
