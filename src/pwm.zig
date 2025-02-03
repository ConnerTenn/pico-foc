const std = @import("std");
const math = std.math;

const pico = @import("pico");
const pwm = pico.hardware.pwm;
const csdk = pico.csdk;
const stdio = pico.stdio;

const bldc = @import("bldc.zig");
const foc = bldc.foc;

pub const PwmDriver = struct {
    const Self = @This();

    u_axis_pins: pwm.PwmSlice,
    v_axis_pins: pwm.PwmSlice,
    w_axis_pins: pwm.PwmSlice,

    pub fn create(u_axis_slice: pwm.PwmSlice.SliceNum, v_axis_slice: pwm.PwmSlice.SliceNum, w_axis_slice: pwm.PwmSlice.SliceNum) Self {
        return Self{
            .u_axis_pins = pwm.PwmSlice.create(u_axis_slice, 0x0FFF),
            .v_axis_pins = pwm.PwmSlice.create(v_axis_slice, 0x0FFF),
            .w_axis_pins = pwm.PwmSlice.create(w_axis_slice, 0x0FFF),
        };
    }

    pub fn init(self: Self) void {
        self.u_axis_pins.init();
        self.v_axis_pins.init();
        self.w_axis_pins.init();

        pwm.enableSlices(&[_]pwm.PwmSlice{
            self.u_axis_pins,
            self.v_axis_pins,
            self.w_axis_pins,
        });
    }

    fn setPwmFromVoltages(self: Self, voltages: foc.PhaseVoltage) void {
        // stdio.print("{}\n", .{voltages});
        self.u_axis_pins.setLevel(rescaleAsInt(u16, math.clamp(voltages.u_axis, -1, 1), self.u_axis_pins.counter_wrap));
        self.v_axis_pins.setLevel(rescaleAsInt(u16, math.clamp(voltages.v_axis, -1, 1), self.u_axis_pins.counter_wrap));
        self.w_axis_pins.setLevel(rescaleAsInt(u16, math.clamp(voltages.w_axis, -1, 1), self.u_axis_pins.counter_wrap));
    }

    pub fn setTorque(self: Self, direct_torque: f32, tangent_torque: f32, angle: f32) void {
        const voltages = foc.getPhaseVoltage(direct_torque, tangent_torque, angle);
        // stdio.print("{}", .{voltages});
        self.setPwmFromVoltages(voltages);
    }
};

pub fn rescaleAsInt(T: anytype, val: f32, max_int: T) T {
    //T must be an unsigned integer
    //Expects val to be in the domain [-1 : 1]
    return @intFromFloat(@as(f32, @floatFromInt(max_int)) * (val + 1.0) / 2.0);
}
