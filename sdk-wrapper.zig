const std = @import("std");
const builtin = @import("builtin");

pub const csdk = import: {
    //Check if currently targeting the RP2040
    if (builtin.cpu.arch.isThumb()) {
        //Import the pico SDK headers
        break :import @cImport({
            @cDefine("LIB_PICO_ATOMIC", "1");
            @cDefine("LIB_PICO_BIT_OPS", "1");
            @cDefine("LIB_PICO_BIT_OPS_PICO", "1");
            @cDefine("LIB_PICO_CLIB_INTERFACE", "1");
            @cDefine("LIB_PICO_CRT0", "1");
            @cDefine("LIB_PICO_CXX_OPTIONS", "1");
            @cDefine("LIB_PICO_DIVIDER", "1");
            @cDefine("LIB_PICO_DIVIDER_HARDWARE", "1");
            @cDefine("LIB_PICO_DOUBLE", "1");
            @cDefine("LIB_PICO_DOUBLE_PICO", "1");
            @cDefine("LIB_PICO_FLOAT", "1");
            @cDefine("LIB_PICO_FLOAT_PICO", "1");
            @cDefine("LIB_PICO_INT64_OPS", "1");
            @cDefine("LIB_PICO_INT64_OPS_PICO", "1");
            @cDefine("LIB_PICO_MALLOC", "1");
            @cDefine("LIB_PICO_MEM_OPS", "1");
            @cDefine("LIB_PICO_MEM_OPS_PICO", "1");
            @cDefine("LIB_PICO_NEWLIB_INTERFACE", "1");
            @cDefine("LIB_PICO_PLATFORM", "1");
            @cDefine("LIB_PICO_PLATFORM_COMPILER", "1");
            @cDefine("LIB_PICO_PLATFORM_PANIC", "1");
            @cDefine("LIB_PICO_PLATFORM_SECTIONS", "1");
            @cDefine("LIB_PICO_PRINTF", "1");
            @cDefine("LIB_PICO_PRINTF_PICO", "1");
            @cDefine("LIB_PICO_RUNTIME", "1");
            @cDefine("LIB_PICO_RUNTIME_INIT", "1");
            @cDefine("LIB_PICO_STANDARD_BINARY_INFO", "1");
            @cDefine("LIB_PICO_STANDARD_LINK", "1");
            @cDefine("LIB_PICO_STDIO", "1");
            @cDefine("LIB_PICO_STDIO_UART", "1");
            @cDefine("LIB_PICO_STDLIB", "1");
            @cDefine("LIB_PICO_SYNC", "1");
            @cDefine("LIB_PICO_SYNC_CRITICAL_SECTION", "1");
            @cDefine("LIB_PICO_SYNC_MUTEX", "1");
            @cDefine("LIB_PICO_SYNC_SEM", "1");
            @cDefine("LIB_PICO_TIME", "1");
            @cDefine("LIB_PICO_TIME_ADAPTER", "1");
            @cDefine("LIB_PICO_UTIL", "1");
            @cDefine("PICO_32BIT", "1");
            @cDefine("PICO_BOARD", "pico");
            @cDefine("PICO_BUILD", "1");
            @cDefine("PICO_CMAKE_BUILD_TYPE", "Release");
            @cDefine("PICO_COPY_TO_RAM", "0");
            @cDefine("PICO_CXX_ENABLE_EXCEPTIONS", "0");
            @cDefine("PICO_NO_FLASH", "0");
            @cDefine("PICO_NO_HARDWARE", "0");
            @cDefine("PICO_ON_DEVICE", "1");
            @cDefine("PICO_RP2040", "1");
            @cDefine("PICO_TARGET_NAME", "blink");
            @cDefine("PICO_USE_BLOCKED_RAM", "0");

            // @cInclude("hardware/gpio.h");
            @cInclude("stdio.h");
            @cInclude("pico/stdlib.h");
            // @cInclude("pico/time.h");
        });
    }
    //We must be in a test environment
    else {
        //Import a dummy SDK since we can't link with the real SDK
        break :import @import("test-sdk.zig");
    }
};

pub const GPIO_IN = false;
pub const GPIO_OUT = true;

pub const GPIO_HIGH = true;
pub const GPIO_LOW = false;

pub const LED_PIN = csdk.PICO_DEFAULT_LED_PIN;
