
cmake_minimum_required(VERSION 3.13)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

set(PICO_SDK_PATH "./pico-sdk")

include(./pico-sdk/pico_sdk_init.cmake)

project(motor-demo C CXX ASM)
pico_sdk_init()

add_executable(${PROJECT_NAME} 
    template.c
)

pico_enable_stdio_uart(${PROJECT_NAME} 0)
pico_enable_stdio_usb(${PROJECT_NAME} 1)

# Add the standard library to the build
target_link_libraries(${PROJECT_NAME} pico_stdlib)# ${CMAKE_SOURCE_DIR}/zig-out/lib/libbldc.a)

pico_add_extra_outputs(${PROJECT_NAME})
