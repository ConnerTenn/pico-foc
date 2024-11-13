
#== Common setup ==
cmake_minimum_required(VERSION 3.13)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

set(PICO_SDK_PATH "./pico-sdk")

include(./pico-sdk/pico_sdk_init.cmake)
pico_sdk_init()


#== Application ==
project(motor-demo C CXX ASM)
set(CMAKE_VERBOSE_MAKEFILE ON)
# pico_sdk_init()

add_executable(${PROJECT_NAME}
    # blink.c
)

pico_enable_stdio_uart(${PROJECT_NAME} 0)
pico_enable_stdio_usb(${PROJECT_NAME} 1)

# Add libraries to the build
add_library(zig_library STATIC IMPORTED)
set_property(TARGET zig_library PROPERTY IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/zig-out/lib/libbldc.a)

target_link_libraries(${PROJECT_NAME} zig_library pico_stdlib hardware_spi)

pico_add_extra_outputs(${PROJECT_NAME})
