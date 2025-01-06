const std = @import("std");
const math = std.math;

const bldc = @import("bldc.zig");
const stdio = bldc.stdio;
const csdk = bldc.csdk;
const foc = bldc.foc;

// const duty_cycle_pio = @cImport({
//     @cInclude("duty_cycle.pio.h");
// });

pub const Pio = struct {
    const Self = @This();

    pio_obj: csdk.PIO,
    state_machine: c_uint,
    offset: c_uint,
    gpio_base: c_uint,
    gpio_count: c_uint,

    pub fn create(program: *const csdk.pio_program_t, gpio_base: c_uint, gpio_count: c_uint) Self {
        var state_machine: c_uint = undefined;
        var pio_obj: csdk.PIO = undefined;
        var offset: c_uint = undefined;
        _ = csdk.pio_claim_free_sm_and_add_program_for_gpio_range(program, &pio_obj, &state_machine, &offset, gpio_base, gpio_count, true);

        const self = Self{
            .pio_obj = pio_obj,
            .state_machine = state_machine,
            .offset = offset,
            .gpio_base = gpio_base,
            .gpio_count = gpio_count,
        };

        return self;
    }
};

pub const DutyCycle = struct {
    const Self = @This();

    // pio_obj: csdk.PIO,
    // state_machine: c_uint,
    pio: Pio,

    pub fn create(gpio_base: c_uint, gpio_count: c_uint) Self {
        // _ = csdk.pio_claim_free_sm_and_add_program_for_gpio_range(program, &pio_obj, &state_machine, &offset, gpio_base, gpio_count, true);
        const pio = Pio.create(&csdk.duty_cycle_program, gpio_base, gpio_count);

        return Self{
            .pio = pio,
        };
    }

    pub fn init(self: Self) void {
        var state_machine_config: csdk.pio_sm_config = csdk.duty_cycle_program_get_default_config(self.pio.offset);
        csdk.sm_config_set_set_pins(&state_machine_config, self.pio.gpio_base, self.pio.gpio_count);
        _ = csdk.pio_sm_init(self.pio.pio_obj, self.pio.state_machine, self.pio.offset, &state_machine_config);
        csdk.pio_sm_set_enabled(self.pio.pio_obj, self.pio.state_machine, true);
    }

    pub fn readDutyCycle(self: Self) f32 {
        const sample = csdk.pio_sm_get_blocking(self.pio.pio_obj, self.pio.state_machine);
        return @floatFromInt(sample);
    }
};
