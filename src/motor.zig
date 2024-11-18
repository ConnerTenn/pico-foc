const std = @import("std");
const math = std.math;
const tau = math.tau;

const bldc = @import("bldc.zig");
const csdk = bldc.csdk;
const stdio = bldc.stdio;
const pwm = bldc.pwm;
const foc = bldc.foc;

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

    driver: pwm.PwmDriver,
    windings_per_rotation: u8,

    sensor: bldc.sensor.LIS3MDL,

    target: Parameters(?f32) = .{ .angle = null, .velocity = null, .torque = null, .acceleration = null },
    limits: Parameters(?f32) = .{ .angle = null, .velocity = null, .torque = null, .acceleration = null },
    state: Parameters(f32) = .{ .angle = 0, .velocity = 0, .torque = 0, .acceleration = 0 },

    sensor_angle: f32 = 0,
    sensor_angle_bias: f32,

    last_time_us: csdk.absolute_time_t = 0,

    pub fn create(u_axis_slice: pwm.Slice, v_axis_slice: pwm.Slice, w_axis_slice: pwm.Slice, windings_per_rotation: u8, sensor: bldc.sensor.LIS3MDL) Self {
        return Self{
            .driver = pwm.PwmDriver.create(u_axis_slice, v_axis_slice, w_axis_slice),
            .windings_per_rotation = windings_per_rotation,
            .sensor = sensor,
            .sensor_angle_bias = 0,
        };
    }

    pub fn init(self: *Self) void {
        self.driver.init();
        self.sensor.init();

        self.setTorque(1.0, 0.0, 0);
        csdk.sleep_ms(500);

        self.sensor_angle_bias = self.sensor.getAngle();
        stdio.print("sensor angle bias: {d:.3}\n", .{self.sensor_angle_bias / math.tau});
    }

    pub fn setTorque(self: Self, direct_torque: f32, tangent_torque: f32, angle: f32) void {
        self.driver.setTorque(direct_torque, tangent_torque, angle * @as(f32, @floatFromInt(self.windings_per_rotation)));
    }

    pub fn setPosition(self: Self, angle: f32) void {
        _ = self; // autofix
        _ = angle; // autofix
    }

    pub fn setRate(self: Self, rad_per_sec: f32) void {
        _ = self; // autofix
        _ = rad_per_sec; // autofix
    }

    pub fn update(self: *Self) void {
        const current_time_us = csdk.get_absolute_time();
        const delta_time_us = current_time_us - self.last_time_us;
        const delta_time_s: f32 = @as(f32, @floatFromInt(delta_time_us)) / (1000.0 * 1000.0);

        self.sensor_angle = math.mod(f32, self.sensor.getAngle() - self.sensor_angle_bias, tau) catch 0.0;
        stdio.print("{}   \r", .{self});

        // if (self.target.velocity) |velocity| {
        //     self.state.angle = self.state.angle + velocity * delta_time_s;
        // }

        // if (self.target.torque) |torque| {
        //     self.state.torque = torque;
        // } else {
        //     self.state.torque = 1;
        // }

        // self.setTorque(self.state.torque, 0, self.state.angle);
        const target_angle = 0.5 * tau;

        const delta_error = target_angle - self.sensor_angle;
        // const abs_error = @abs(delta_error);

        // if (delta_error > 0) {
        //     //Need to increase sensor angle
        //     self.state.angle += 0.5 * tau * delta_time_s;
        // } else if (delta_error < 0) {
        //     //Need to decrase sensor angle
        //     self.state.angle -= 0.5 * tau * delta_time_s;
        // }
        self.state.angle += delta_error * tau * delta_time_s;

        self.setTorque(0.2, 0, self.state.angle);

        // if (self.target.angle) |target_angle| {
        //     var torque: f32 = 0.0;

        //     if (target_angle > self.state.angle) {
        //         torque = 1.0;
        //     }
        // }
        // self.setTorque(0.1, 0.0, self.state.angle + 0.1 * tau);

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
