const std = @import("std");
const math = std.math;

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

    last_time_us: csdk.absolute_time_t = 0,

    pub fn create(u_axis_slice: pwm.Slice, v_axis_slice: pwm.Slice, w_axis_slice: pwm.Slice, windings_per_rotation: u8, sensor: bldc.sensor.LIS3MDL) Self {
        return Self{
            .driver = pwm.PwmDriver.create(u_axis_slice, v_axis_slice, w_axis_slice),
            .windings_per_rotation = windings_per_rotation,
            .sensor = sensor,
        };
    }

    pub fn init(self: Self) void {
        self.driver.init();
        self.sensor.init();
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
        stdio.print("time: {d:.1}s {d:.1}\n", .{ @as(f32, @floatFromInt(current_time_us)) / (1000.0 * 1000.0), self.state.angle / math.tau });

        if (self.target.velocity) |velocity| {
            self.state.angle = self.state.angle + velocity * delta_time_s;
        }

        if (self.target.torque) |torque| {
            self.state.torque = torque;
        } else {
            self.state.torque = 1;
        }

        self.setTorque(self.state.torque, 0, self.state.angle);

        self.last_time_us = current_time_us;
    }
};

pub fn run() noreturn {
    csdk.gpio_set_function(8, csdk.GPIO_FUNC_PWM);
    csdk.gpio_set_function(9, csdk.GPIO_FUNC_PWM);
    csdk.gpio_set_function(12, csdk.GPIO_FUNC_PWM);
    csdk.gpio_set_function(13, csdk.GPIO_FUNC_PWM);
    csdk.gpio_set_function(14, csdk.GPIO_FUNC_PWM);
    csdk.gpio_set_function(15, csdk.GPIO_FUNC_PWM);

    var motor = Motor.create(4, 6, 7, 8);
    motor.init();
    motor.setTorque(1, 0, 0);

    csdk.sleep_ms(500);

    motor.target.velocity = math.tau * 1;
    motor.target.torque = 1;
    while (true) {
        motor.update();
    }
}
