
PROJECT_NAME := motor-demo
BUILD_DIR := ${CURDIR}/build

PICO_TARGET := rp2350
EXTRA_LIB_DEPENDENCIES := ${BUILD_DIR}/duty_cycle.pio.h

include Pico-Zig/rules.mk


${BUILD_DIR}/duty_cycle.pio.h: pico-sdk src/duty_cycle.pio | ${BUILD_DIR}
	@cd $(BUILD_DIR) && cmake .. -DPICO_SDK_PATH=${RUN_DIR}/pico-sdk -DPICO_BOARD=pico2 -DPICO_PLATFORM=rp2350 && make -j 20 motor-demo_duty_cycle_pio_h

