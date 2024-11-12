const std = @import("std");
const math = std.math;

const bldc = @import("bldc.zig");
const stdio = bldc.stdio;
const csdk = bldc.csdk;
const foc = bldc.foc;

pub const Slice = c_uint;

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
