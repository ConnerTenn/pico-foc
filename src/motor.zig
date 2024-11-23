const std = @import("std");
const math = std.math;
const tau = math.tau;

const bldc = @import("bldc.zig");
const csdk = bldc.csdk;
const stdio = bldc.stdio;
const pwm = bldc.pwm;
const foc = bldc.foc;

pub fn deltaError(T: type, measured: T, target: T, modulo: T) T {
    return bldc.mod(T, target - measured + modulo / 2.0, modulo, .regular) - modulo / 2.0;
}

pub const Motor = struct {
    const Self = @This();

    pub fn Parameters(T: type) type {
        return struct {
            angle: T, //rad
            velocity: T, // rad/sec
            torque: T, // [0,1]
            acceleration: T, // rad/(s^2)
        };
    }

    const num_calibration_samples = 128;

    driver: pwm.PwmDriver,
    windings_per_rotation: u8,

    sensor: bldc.sensor.LIS3MDL,

    calibration_data: [num_calibration_samples]f32 = .{0} ** num_calibration_samples,

    target: Parameters(?f32) = .{ .angle = null, .velocity = null, .torque = null, .acceleration = null },
    limits: Parameters(?f32) = .{ .angle = null, .velocity = null, .torque = null, .acceleration = null },
    state: Parameters(f32) = .{ .angle = 0, .velocity = 0, .torque = 0, .acceleration = 0 },

    sensor_angle: f32 = 0,
    pid: PIDcontrol,

    last_time_us: csdk.absolute_time_t = 0,

    pub fn create(u_axis_slice: pwm.Slice, v_axis_slice: pwm.Slice, w_axis_slice: pwm.Slice, windings_per_rotation: u8, sensor: bldc.sensor.LIS3MDL, pid: PIDcontrol) Self {
        return Self{
            .driver = pwm.PwmDriver.create(u_axis_slice, v_axis_slice, w_axis_slice),
            .windings_per_rotation = windings_per_rotation,
            .sensor = sensor,
            .pid = pid,
        };
    }

    pub fn init(self: *Self) void {
        self.driver.init();
        self.sensor.init();

        self.setTorque(1.0, 0.0, 0);

        //Settling time before measuring the angle bias
        csdk.sleep_ms(1000);

        self.calibrate();

        //Update the last time
        self.last_time_us = csdk.get_absolute_time();
    }

    pub fn calibrate(self: *Self) void {
        stdio.print("Running Calibration...\n", .{});

        //Loop through every sample
        for (0..num_calibration_samples) |sample_idx| {
            //Drive to the target angle
            const target_angle = tau * @as(f32, @floatFromInt(sample_idx)) / @as(f32, @floatFromInt(num_calibration_samples));

            self.setTorque(1.0, 0.0, target_angle);
            csdk.sleep_ms(100);

            //Collect a number of samples and average them
            const num_samples = 10;
            var measured_angle: f32 = 0;
            for (0..num_samples) |_| {
                measured_angle += self.sensor.getAngle();
            }
            measured_angle = measured_angle / @as(f32, @floatFromInt(num_samples));

            self.calibration_data[sample_idx] = target_angle - measured_angle;
        }

        stdio.print("Calibration samples:\n", .{});
        for (0..num_calibration_samples) |sample_idx| {
            const target_angle = tau * @as(f32, @floatFromInt(sample_idx)) / @as(f32, @floatFromInt(num_calibration_samples));
            stdio.print("{d: >6.3} rad: {d: >6.3}\n", .{ target_angle, self.calibration_data[sample_idx] });
        }
    }

    pub fn setTorque(self: Self, direct_torque: f32, tangent_torque: f32, angle: f32) void {
        self.driver.setTorque(direct_torque, tangent_torque, angle * @as(f32, @floatFromInt(self.windings_per_rotation)));
    }

    pub fn getAngle(self: Self) f32 {
        const raw_angle = self.sensor.getAngle();
        // stdio.print("raw_angle:{d} ", .{raw_angle});

        //Offset to apply so that the selected sample is in the middle of the angle range
        const offset_for_centered_sample = tau / @as(f32, @floatFromInt(num_calibration_samples));
        const sample_bin = @as(f32, @floatFromInt(num_calibration_samples)) * raw_angle / tau + offset_for_centered_sample;
        const sample_idx = @mod(@as(u16, @intFromFloat(sample_bin)), num_calibration_samples);
        // stdio.print("sample_idx:{} ", .{sample_idx});

        //Get the compesated angle using the calibration data
        const compensated_angle = raw_angle + self.calibration_data[sample_idx];

        // stdio.print("compensated_angle:{d}\n", .{compensated_angle});
        return bldc.mod(f32, compensated_angle, tau, .regular);
    }

    pub fn setPosition(self: Self, angle: f32) void {
        _ = self; // autofix
        _ = angle; // autofix
    }

    pub fn setRate(self: Self, rad_per_sec: f32) void {
        _ = self; // autofix
        _ = rad_per_sec; // autofix
    }

    pub inline fn update(self: *Self) void {
        const current_time_us = csdk.get_absolute_time();
        const delta_time_us = current_time_us - self.last_time_us;
        const delta_time_s: f32 = @as(f32, @floatFromInt(delta_time_us)) / (1000.0 * 1000.0);

        self.sensor_angle = self.getAngle();
        self.state.angle = self.sensor_angle;

        const target_angle = 0.0 * tau;
        const delta_error = deltaError(f32, self.sensor_angle, target_angle, tau);

        // const torque = (1.0 - math.pow(f32, 1000.0, -@abs(delta_error))) * math.sign(delta_error);

        var torque = self.pid.update(delta_error, delta_time_s);
        if (torque > 1.0) {
            torque = 1.0;
        } else if (torque < -1.0) {
            torque = -1.0;
        }

        self.setTorque(0.0, torque, self.state.angle);

        self.last_time_us = current_time_us;
    }

    pub fn format(self: Self, comptime fmt: []const u8, options: std.fmt.FormatOptions, writer: anytype) !void {
        _ = fmt;
        _ = options;

        try writer.print("Position: {d:.3} ", .{self.sensor_angle / tau});
        try bldc.printPositionGraph(
            30,
            self.sensor_angle,
            0,
            tau,
            writer,
        );
        try writer.print("  state: {d:.3} ", .{self.state.angle / tau});
    }
};

pub const PIDcontrol = struct {
    const Self = @This();

    proportional_gain: f32,
    integral_gain: f32,
    derivative_gain: f32,

    last_error: f32 = 0,
    error_accumulator: f32 = 0,

    pub fn create(proportional_gain: f32, integral_gain: f32, derivative_gain: f32) Self {
        return Self{
            .proportional_gain = proportional_gain,
            .integral_gain = integral_gain,
            .derivative_gain = derivative_gain,
        };
    }

    pub fn update(self: *Self, delta_error: f32, delta_time_s: f32) f32 {
        //accumulate the error for the integral term
        self.error_accumulator += delta_error;

        //Calculate the three components
        const proportional = delta_error * self.proportional_gain;
        const derivative = (delta_error - self.last_error) / delta_time_s * self.derivative_gain;
        const integral = self.error_accumulator / delta_time_s * self.integral_gain;

        //Record the last error for the derivative term
        self.last_error = delta_error;

        return proportional + derivative + integral;
    }
};
