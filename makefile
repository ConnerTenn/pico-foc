
DIR := $(CURDIR)

BIN = motor-demo.uf2
PICO_DEV = /dev/disk/by-label/RPI-RP2
MNT_DIR = /mnt

help:
	@echo "Commands:"
	@echo "  env"
	@echo "  build"
	@echo "  program"
	@echo "  serial"
	@echo "  clean"
	@echo

env:
	nix develop -c $$SHELL

build: build/$(BIN)

program: build/$(BIN)
	sudo mount -o uid=$(shell id -u),gid=$(shell id -g) $(PICO_DEV) $(MNT_DIR)
	cp build/$(BIN) $(MNT_DIR)
	sudo umount $(MNT_DIR)

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

build/$(BIN): zig-out/lib/libbldc.a | pico-sdk
	@mkdir -p build
	@cd build && PICO_SDK_PATH=$(CURDIR)/pico-sdk cmake .. && make -j 20
	@echo
	@echo Done
