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
            .u_axis_pins = PwmPair.create(u_axis_slice),
            .v_axis_pins = PwmPair.create(v_axis_slice),
            .w_axis_pins = PwmPair.create(w_axis_slice),
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
        self.u_axis_pins.setLevel(rescaleAsInt(u16, math.clamp(voltages.u_axis, -1, 1)));
        self.v_axis_pins.setLevel(rescaleAsInt(u16, math.clamp(voltages.v_axis, -1, 1)));
        self.w_axis_pins.setLevel(rescaleAsInt(u16, math.clamp(voltages.w_axis, -1, 1)));
    }

    pub fn setTorque(self: Self, direct_torque: f32, tangent_torque: f32, angle: f32) void {
        const voltages = foc.getPhaseVoltage(direct_torque, tangent_torque, angle);
        self.setPwmFromVoltages(voltages);
    }
};

pub const PwmPair = struct {
    const Self = @This();

    slice: Slice,

    pub fn create(slice: Slice) Self {
        const self = Self{
            .slice = slice,
        };

        return self;
    }

    pub fn init(self: Self) void {
        var config = csdk.pwm_get_default_config();

        csdk.pwm_config_set_output_polarity(&config, true, false);
        csdk.pwm_config_set_clkdiv_int(&config, 1);
        // csdk.pwm_config_set_phase_correct(&config, true);

        csdk.pwm_init(self.slice, &config, false);

        csdk.pwm_set_counter(self.slice, 0);
    }

    pub fn disable(self: Self) void {
        csdk.pwm_set_enabled(self.slice, false);
    }

    pub inline fn setLevel(self: Self, level: u16) void {
        csdk.pwm_set_both_levels(self.slice, level, level);
    }
};

pub fn enableSlices(slices: []const Slice) void {
    var mask: u32 = 0;

    for (slices) |slice| {
        mask = mask | (@as(u32, 1) << @truncate(slice));
    }

    csdk.pwm_set_mask_enabled(mask);
}

pub fn rescaleAsInt(T: anytype, val: f32) T {
    //T must be an unsigned integer
    //Expects val to be in the domain [-1 : 1]
    const max_int = math.maxInt(T);
    return @intFromFloat(@as(f32, @floatFromInt(max_int)) * (val + 1.0) / 2.0);
}
