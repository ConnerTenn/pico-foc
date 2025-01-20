const std = @import("std");
const math = std.math;

const pico = @import("pico");
const csdk = pico.csdk;
const stdio = pico.stdio;
const pio = pico.pio;

const bldc = @import("bldc.zig");
const foc = bldc.foc;

const duty_cycle_pio = @cImport({
    @cInclude("duty_cycle.pio.h");
});

pub const DutyCycle = struct {
    const Self = @This();

    // pio_obj: csdk.PIO,
    // state_machine: c_uint,
    pio_high: pio.Pio,
    pio_low: pio.Pio,

    pub fn create(gpio_base: pico.gpio.Pin, gpio_count: pico.gpio.Pin.Count) Self {
        // _ = csdk.pio_claim_free_sm_and_add_program_for_gpio_range(program, &pio_obj, &state_machine, &offset, gpio_base, gpio_count, true);
        // const pio = Pio.create(&csdk.duty_cycle_program, gpio_base, gpio_count);

        return Self{
            .pio_high = pio.Pio.create(
                @ptrCast(&duty_cycle_pio.high_cycle_program),
                @ptrCast(&duty_cycle_pio.high_cycle_program_get_default_config),
                gpio_base,
                gpio_count,
            ),
            .pio_low = pio.Pio.create(
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
