const std = @import("std");
const math = std.math;

const bldc = @import("bldc.zig");
const stdio = bldc.stdio;
const csdk = bldc.csdk;
const foc = bldc.foc;

pub const Slice = c_uint;

pub const PwmDriver = struct {
    const Self = @This();

    u_axis_pins: PwmPair,
    v_axis_pins: PwmPair,
    w_axis_pins: PwmPair,

    pub fn create(u_axis_slice: Slice, v_axis_slice: Slice, w_axis_slice: Slice) Self {
        return Self{
            .u_axis_pins = PwmPair.create(u_axis_slice, 0x0FFF),
            .v_axis_pins = PwmPair.create(v_axis_slice, 0x0FFF),
            .w_axis_pins = PwmPair.create(w_axis_slice, 0x0FFF),
        };
    }

    pub fn init(self: Self) void {
        self.u_axis_pins.init();
        self.v_axis_pins.init();
        self.w_axis_pins.init();

        enableSlices(&[_]Slice{
            self.u_axis_pins.slice,
            self.v_axis_pins.slice,
            self.w_axis_pins.slice,
        });
    }

    fn setPwmFromVoltages(self: Self, voltages: foc.PhaseVoltage) void {
        // stdio.print("{}\n", .{voltages});
        self.u_axis_pins.setLevel(voltages.u_axis);
        self.v_axis_pins.setLevel(voltages.v_axis);
        self.w_axis_pins.setLevel(voltages.w_axis);
    }

    pub fn setTorque(self: Self, direct_torque: f32, tangent_torque: f32, angle: f32) void {
        const voltages = foc.getPhaseVoltage(direct_torque, tangent_torque, angle);
        // stdio.print("{}\n", .{voltages});
        self.setPwmFromVoltages(voltages);
    }
};

pub const PwmPair = struct {
    const Self = @This();

    slice: Slice,
    counter_wrap: u16,

    pub fn create(slice: Slice, counter_wrap: ?u16) Self {
        const self = Self{
            .slice = slice,
            .counter_wrap = counter_wrap orelse math.maxInt(u16),
        };

        return self;
    }

    pub fn init(self: Self) void {
        var config = csdk.pwm_get_default_config();

        csdk.pwm_config_set_output_polarity(&config, true, false);
        csdk.pwm_config_set_clkdiv_int(&config, 1);
        csdk.pwm_config_set_wrap(&config, self.counter_wrap);
        // csdk.pwm_config_set_phase_correct(&config, true);

        csdk.pwm_init(self.slice, &config, false);

        csdk.pwm_set_counter(self.slice, 0);
    }

    pub fn disable(self: Self) void {
        csdk.pwm_set_enabled(self.slice, false);
    }

    pub inline fn setLevelRaw(self: Self, level: u16) void {
        csdk.pwm_set_both_levels(self.slice, level, level);
    }

    pub inline fn setLevel(self: Self, level: f32) void {
        self.setLevelRaw(
            rescaleAsInt(u16, math.clamp(level, -1, 1), self.counter_wrap),
        );
    }
};

pub fn enableSlices(slices: []const Slice) void {
    var mask: u32 = 0;

    for (slices) |slice| {
        mask = mask | (@as(u32, 1) << @truncate(slice));
    }

    csdk.pwm_set_mask_enabled(mask);
}

pub fn rescaleAsInt(T: anytype, val: f32, max_int: T) T {
    //T must be an unsigned integer
    //Expects val to be in the domain [-1 : 1]
    return @intFromFloat(@as(f32, @floatFromInt(max_int)) * (val + 1.0) / 2.0);
}
