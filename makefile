
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
	rm -rf build/*


zig-out/lib/libzig.a:
	@zig build test && zig build
	@echo

build/motor-demo.uf2: zig-out/lib/libzig.a
	@mkdir -p build
	@cd build && PICO_SDK_PATH=$(CURDIR)/pico-sdk cmake .. && make -j 20
	@rm ./gen/pico/*
	@cp ./build/generated/pico_base/pico/* ./gen/pico
	@echo
	@echo Done
