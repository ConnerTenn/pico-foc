
DIR := $(CURDIR)

BUILD_DIR = $(DIR)/build
BIN = $(BUILD_DIR)/motor-demo.uf2
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

.PHONY:build
build: $(BIN)

program: $(BIN)
	sudo mount -o uid=$(shell id -u),gid=$(shell id -g) $(PICO_DEV) $(MNT_DIR)
	cp $(BIN) $(MNT_DIR)
	sudo umount $(MNT_DIR)

serial:
	sudo picocom -b 115200 /dev/ttyACM0

clean:
	rm -rf zig-out
	rm -rf zig-cache
	rm -rf $(BUILD_DIR)

# Build directory
$(BUILD_DIR):
	@mkdir -p $(BUILD_DIR)

# Zig build
zig-out/lib/libbldc.a: *.zig $(BUILD_DIR)/generated/pico_base/pico
	@zig build test && zig build -freference-trace build
	@echo

# SDK repo
pico-sdk:
	git clone https://github.com/raspberrypi/pico-sdk.git
	cd $@; \
	git submodule update --init

# 
$(BUILD_DIR)/generated/pico_base/pico: CMakeLists.txt | pico-sdk $(BUILD_DIR)
	@cd $(BUILD_DIR) && PICO_SDK_PATH=$(CURDIR)/pico-sdk cmake .. && make -j 20 blink

$(BIN): zig-out/lib/libbldc.a CMakeLists.txt | pico-sdk $(BUILD_DIR)
	@cd $(BUILD_DIR) && PICO_SDK_PATH=$(CURDIR)/pico-sdk cmake .. && make -j 20
	@echo
	@echo Done
