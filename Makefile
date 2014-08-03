tmp=_git_distcheck

all:
	cargo build --release

bugs:
	make -C examples bugs

examples:
	cd examples; cargo build --release

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
