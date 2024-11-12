const std = @import("std");
const math = std.math;

const bldc = @import("bldc.zig");
const csdk = bldc.csdk;
const stdio = bldc.stdio;
const pwm = bldc.pwm;
const foc = bldc.foc;

pub const Motor = struct {
    const Self = @This();

    driver: pwm.PwmDriver,

    pub fn create(u_axis_slice: pwm.Slice, v_axis_slice: pwm.Slice, w_axis_slice: pwm.Slice) Self {
        return Self{
            .driver = pwm.PwmDriver.create(u_axis_slice, v_axis_slice, w_axis_slice),
        };
    }

    pub fn init(self: Self) void {
        self.driver.init();
    }

    pub fn setTorque(self: Self, direct_torque: f32, tangent_torque: f32, angle: f32) void {
        self.driver.setTorque(direct_torque, tangent_torque, angle);
    }

    pub fn setPosition(self: Self, angle: f32) void {
        _ = self; // autofix
        _ = angle; // autofix
    }

    pub fn setRate(self: Self, rad_per_sec: f32) void {
        _ = self; // autofix
        _ = rad_per_sec; // autofix
    }
};

pub fn run() noreturn {
    csdk.gpio_set_function(8, csdk.GPIO_FUNC_PWM);
    csdk.gpio_set_function(9, csdk.GPIO_FUNC_PWM);
    csdk.gpio_set_function(12, csdk.GPIO_FUNC_PWM);
    csdk.gpio_set_function(13, csdk.GPIO_FUNC_PWM);
    csdk.gpio_set_function(14, csdk.GPIO_FUNC_PWM);
    csdk.gpio_set_function(15, csdk.GPIO_FUNC_PWM);

    const motor = Motor.create(4, 6, 7);
    motor.init();
    motor.setTorque(1, 0, 0);

    csdk.sleep_ms(500);

    const acceleration = 0.00000001;
    const max_speed = 0.05;
    var speed: f32 = 0.0;
    var angle: f32 = 0.0;

    while (true) {
        // stdio.print("angle:{}  speed:{}\n", .{ angle, speed });
        motor.setTorque(1, 0, angle);

        angle += speed;
        angle = @mod(angle, math.tau);

        if (speed < max_speed) {
            speed += acceleration;
        } else {
            speed = max_speed;
        }

        // csdk.sleep_us(50);
    }
}
