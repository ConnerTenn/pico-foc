const std = @import("std");
const builtin = @import("builtin");

pub const csdk = import: {
    //Check if currently targeting the RP2040
    if (builtin.cpu.arch.isARM()) {
        //Import the pico SDK headers
        break :import @cImport({
            @cDefine("__unused", "");
            @cInclude("pico/stdlib.h");
            // @cInclude("pico/stdio.h");
            // @cInclude("pico/time.h");
            // @cInclude("hardware/gpio.h");
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
