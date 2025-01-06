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
    pio_high: Pio,
    pio_low: Pio,

    pub fn create(gpio_base: c_uint, gpio_count: c_uint) Self {
        // _ = csdk.pio_claim_free_sm_and_add_program_for_gpio_range(program, &pio_obj, &state_machine, &offset, gpio_base, gpio_count, true);
        // const pio = Pio.create(&csdk.duty_cycle_program, gpio_base, gpio_count);

        return Self{
            .pio_high = Pio.create(&csdk.high_cycle_program, gpio_base, gpio_count),
            .pio_low = undefined, //Pio.create(&csdk.low_cycle_program, gpio_base, gpio_count),
        };
    }

    pub fn init(self: *Self) void {
        //High Cycle
        //Configure the pin direction
        _ = csdk.pio_sm_set_consecutive_pindirs(self.pio_high.pio_obj, self.pio_high.state_machine, self.pio_high.gpio_base, self.pio_high.gpio_count, false);
        //Configure the state machine
        stdio.print("Configure state machine: {}\n", .{self.pio_high.gpio_base});
        var high_cycle_state_machine_config: csdk.pio_sm_config = csdk.high_cycle_program_get_default_config(self.pio_high.offset);
        csdk.sm_config_set_jmp_pin(&high_cycle_state_machine_config, self.pio_high.gpio_base);
        //Start the state machine
        stdio.print("start state machine: {}\n", .{self.pio_high.gpio_base});
        _ = csdk.pio_sm_init(self.pio_high.pio_obj, self.pio_high.state_machine, self.pio_high.offset, &high_cycle_state_machine_config);
        csdk.pio_sm_set_enabled(self.pio_high.pio_obj, self.pio_high.state_machine, true);

        // //Low Cycle
        // //Configure the state machine
        // var low_cycle_state_machine_config: csdk.pio_sm_config = csdk.low_cycle_program_get_default_config(self.pio_low.offset);
        // _ = csdk.pio_sm_init(self.pio_low.pio_obj, self.pio_low.state_machine, self.pio_low.offset, &low_cycle_state_machine_config);
        // //Start the state machine
        // csdk.pio_sm_set_enabled(self.pio_low.pio_obj, self.pio_low.state_machine, true);
    }

    pub fn readDutyCycle(self: *Self) f32 {
        const high_time = csdk.pio_sm_get_blocking(self.pio_high.pio_obj, self.pio_high.state_machine);
        // const low_time = csdk.pio_sm_get_blocking(self.pio_low.pio_obj, self.pio_low.state_machine);
        return @floatFromInt(high_time);
    }
};
