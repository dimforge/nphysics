tmp=_git_distcheck

all:
	cargo build --features "3df32"
	cargo build --features "2df32"
	cargo build --features "2df64"
	cargo build --features "3df64"

examples:
	cd examples3; cargo build --release
	cd examples2; cargo build --release

distcheck:
	rm -rf $(tmp)
	git clone . $(tmp)
	make -C $(tmp)
	make -C $(tmp) examples
	rm -rf $(tmp)

doc:
	cargo doc

clean:
	cargo clean

.PHONY:doc
# FIXME: uggly!
.PHONY:examples
.PHONY:bugs
