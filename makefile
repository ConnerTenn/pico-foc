
DIR := $(CURDIR)

help:
	@echo "Commands:"
	@echo "  env"
	@echo "  build"
	@echo

env:
	nix develop
