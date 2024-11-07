
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

clean-all: clean
	rm -rf pico-sdk
	rm -rf Arduino-FOC

# Build directory
$(BUILD_DIR):
	@mkdir -p $(BUILD_DIR)

# Zig build
zig-out/lib/libbldc.a: *.zig $(BUILD_DIR)/generated/pico_base/pico
	@zig build test && zig build -freference-trace build
	@echo

# == Repos ==
pico-sdk:
	git clone https://github.com/raspberrypi/pico-sdk.git
	cd $@; \
	git submodule update --init

Arduino-FOC:
	git clone https://github.com/simplefoc/Arduino-FOC.git

# == CMAKE rules ==
test: $(BUILD_DIR)/generated/pico_base/pico
$(BUILD_DIR)/generated/pico_base/pico: CMakeLists.txt | pico-sdk $(BUILD_DIR)
	@cd $(BUILD_DIR) && PICO_SDK_PATH=$(CURDIR)/pico-sdk cmake .. && make -j 20 depend

$(BIN): zig-out/lib/libbldc.a CMakeLists.txt | pico-sdk Arduino-FOC $(BUILD_DIR)
	@cd $(BUILD_DIR) && PICO_SDK_PATH=$(CURDIR)/pico-sdk cmake .. && make -j 20 motor-demo
	@echo
	@echo == Done ==
	@echo
