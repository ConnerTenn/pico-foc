
PROJECT_NAME := motor-demo
BUILD_DIR := ${CURDIR}/build

PICO_TARGET := rp2350
EXTRA_LIB_DEPENDENCIES := ${BUILD_DIR}/duty_cycle.pio.h

include Pico-Zig/rules.mk


${BUILD_DIR}/duty_cycle.pio.h: pico-sdk Pico-Zig/library/duty_cycle.pio | ${BUILD_DIR}
	@cd $(BUILD_DIR) && cmake .. -DPICO_SDK_PATH=${RUN_DIR}/pico-sdk && make -j 20 motor-demo_duty_cycle_pio_h


# == Repos ==
pico-examples:
	git clone https://github.com/raspberrypi/pico-examples.git

