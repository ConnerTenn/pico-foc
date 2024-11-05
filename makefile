
DIR := $(CURDIR)

help:
	@echo "Commands:"
	@echo "  env"
	@echo "  build"
	@echo "  serial"
	@echo "  clean"
	@echo

env:
	nix develop -c $$SHELL

build: build/motor-demo.uf2

serial:
	sudo picocom -b 115200 /dev/ttyACM0

clean:
	rm -rf zig-out
	rm -rf zig-cache
	rm -rf build


zig-out/lib/libbldc.a:
	@zig build test && zig build build
	@echo

pico-sdk:
	git clone https://github.com/raspberrypi/pico-sdk.git
	cd $@; \
	git submodule update --init

build/motor-demo.uf2: zig-out/lib/libbldc.a | pico-sdk
	@mkdir -p build
	@cd build && PICO_SDK_PATH=$(CURDIR)/pico-sdk cmake .. && make -j 20
	@echo
	@echo Done
