const std = @import("std");
const math = std.math;

const pico = @import("pico");
const csdk = pico.csdk;
const stdio = pico.stdio;
const pio = pico.pio;

const bldc = @import("bldc.zig");
const foc = bldc.foc;

const duty_cycle_pio = @cImport({
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
    @cDefine("PICO_RP2040", "0");
    @cDefine("PICO_RP350", "1");
    @cDefine("PICO_TARGET_NAME", "blink");
    @cDefine("PICO_USE_BLOCKED_RAM", "0");

    @cInclude("duty_cycle.pio.h");
});

pub const DutyCycle = struct {
    const Self = @This();

    // pio_obj: csdk.PIO,
    // state_machine: c_uint,
    pio_high: pio.Pio,
    pio_low: pio.Pio,

    pub fn create(gpio_base: pico.gpio.Pin, gpio_count: pico.gpio.Pin.Count) pio.Pio.Error!Self {
        // _ = csdk.pio_claim_free_sm_and_add_program_for_gpio_range(program, &pio_obj, &state_machine, &offset, gpio_base, gpio_count, true);
        // const pio = Pio.create(&csdk.duty_cycle_program, gpio_base, gpio_count);

        return Self{
            .pio_high = try pio.Pio.create(
                @ptrCast(&duty_cycle_pio.high_cycle_program),
                @ptrCast(&duty_cycle_pio.high_cycle_program_get_default_config),
                gpio_base,
                gpio_count,
            ),
            .pio_low = try pio.Pio.create(
                @ptrCast(&duty_cycle_pio.low_cycle_program),
                @ptrCast(&duty_cycle_pio.high_cycle_program_get_default_config),
                gpio_base,
                gpio_count,
            ),
        };
    }

    pub fn init(self: *Self) void {
        //High Cycle
        //Configure the pin direction
        self.pio_high.setConsecutivePinDirs(self.pio_high.gpio_base, self.pio_high.gpio_count, false);
        //Configure the state machine
        var pio_high_config = self.pio_high.getDefaultConfig();
        pio_high_config.setJmpPin(self.pio_high.gpio_base);
        //Start the state machine
        self.pio_high.init(pio_high_config);
        self.pio_high.enable();

        //Low Cycle
        //Configure the pin direction
        self.pio_low.setConsecutivePinDirs(self.pio_low.gpio_base, self.pio_low.gpio_count, false);
        //Configure the state machine
        var pio_low_config = self.pio_low.getDefaultConfig();
        pio_low_config.setJmpPin(self.pio_low.gpio_base);
        //Start the state machine
        self.pio_low.init(pio_low_config);
        self.pio_low.enable();
    }

    pub fn readDutyCycle(self: *Self) f32 {
        const high_time = math.maxInt(u32) - csdk.pio_sm_get_blocking(self.pio_high.pio_obj, self.pio_high.state_machine);
        const low_time = math.maxInt(u32) - csdk.pio_sm_get_blocking(self.pio_low.pio_obj, self.pio_low.state_machine);
        const ratio = @as(f32, @floatFromInt(high_time)) / @as(f32, @floatFromInt(high_time + low_time));
        // stdio.print("high:low {d: >7}:{d: >7}   {d:.4}\n", .{ high_time, low_time, ratio });
        return ratio;
    }

    pub fn getSensor(self: *Self) bldc.sensor.Sensor {
        return bldc.sensor.Sensor{
            .ctx = self,
            .getAngleFn = _getAngle,
        };
    }

    fn _getAngle(ctx: *anyopaque) f32 {
        const self: *Self = @alignCast(@ptrCast(ctx));
        return math.tau - self.readDutyCycle() * math.tau;
    }
};
