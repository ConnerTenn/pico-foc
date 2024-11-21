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

    calibration_data: [num_calibration_samples]f32 = undefined,

    target: Parameters(?f32) = .{ .angle = null, .velocity = null, .torque = null, .acceleration = null },
    limits: Parameters(?f32) = .{ .angle = null, .velocity = null, .torque = null, .acceleration = null },
    state: Parameters(f32) = .{ .angle = 0, .velocity = 0, .torque = 0, .acceleration = 0 },

    sensor_angle: f32 = 0,

    last_time_us: csdk.absolute_time_t = 0,

    pub fn create(u_axis_slice: pwm.Slice, v_axis_slice: pwm.Slice, w_axis_slice: pwm.Slice, windings_per_rotation: u8, sensor: bldc.sensor.LIS3MDL) Self {
        return Self{
            .driver = pwm.PwmDriver.create(u_axis_slice, v_axis_slice, w_axis_slice),
            .windings_per_rotation = windings_per_rotation,
            .sensor = sensor,
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
            csdk.sleep_ms(200);

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

        const new_sensor_angle = self.getAngle();
        //Smoothing?
        self.sensor_angle = self.sensor_angle * 0.0 + new_sensor_angle * 1.0;

        const demo: enum { angle_monitor, offset_measurement, tracking_pos } = .offset_measurement;

        switch (demo) {
            .angle_monitor => {
                const DeltaErrorStats = struct {
                    var min_error: f32 = math.floatMax(f32);
                    var max_error: f32 = 0;
                };

                const delta_error = deltaError(f32, self.sensor_angle, self.state.angle, tau);

                DeltaErrorStats.min_error = @min(DeltaErrorStats.min_error, delta_error);
                DeltaErrorStats.max_error = @max(DeltaErrorStats.max_error, delta_error);

                stdio.print("target:{d:.3} measured:{d:.3}  delta:{d: >6.3}[{d: >6.3}->{d: >6.3}]  \r", .{
                    self.state.angle / tau,
                    self.sensor_angle / tau,
                    delta_error / tau,
                    DeltaErrorStats.min_error / tau,
                    DeltaErrorStats.max_error / tau,
                });

                self.setTorque(1.0, 0.0, self.state.angle);

                // if (current_time_us < 5 * 1000 * 1000) {
                self.state.angle = bldc.mod(f32, self.state.angle + 0.8 * tau * delta_time_s, tau, .regular);
                // }
            },

            .tracking_pos => {
                const target_angle = 0.0 * tau;

                var delta_error = deltaError(f32, self.sensor_angle, target_angle, tau);
                delta_error = @min(delta_error, tau / 4.0);
                delta_error = @max(delta_error, -tau / 4.0);

                // if (delta_error > 0.1) {
                //     delta_error = 0.1;
                // } else if (delta_error < -0.1) {
                //     delta_error = -0.1;
                // }

                self.state.angle = self.sensor_angle + delta_error * delta_time_s;
                // stdio.print("state:{d:.3} sensor:{d:.3} target:{d:.3} dE:{d:.3} \r", .{
                //     self.state.angle / tau,
                //     self.sensor_angle / tau,
                //     target_angle / tau,
                //     delta_error / tau,
                // });

                self.setTorque(1.0, 0.0, self.state.angle);
            },

            .offset_measurement => {
                const sample_depth = 20;
                const OffsetData = struct {
                    var sample_data: [sample_depth]f32 = .{0} ** sample_depth;
                };

                const delta_error = deltaError(f32, self.sensor_angle, self.state.angle, tau);
                const sample_idx: u8 = @intFromFloat(@as(f32, @floatFromInt(sample_depth)) * bldc.mod(f32, self.state.angle, tau, .regular) / tau);
                OffsetData.sample_data[sample_idx] = delta_error;

                for (OffsetData.sample_data) |sample| {
                    stdio.print("{d: >5.2} ", .{sample});
                }
                stdio.print("\r", .{});

                self.setTorque(1.0, 0.0, self.state.angle);

                // if (current_time_us < 5 * 1000 * 1000) {
                self.state.angle = bldc.mod(f32, self.state.angle + 0.1 * tau * delta_time_s, tau, .regular);
            },
        }

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
