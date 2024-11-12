const std = @import("std");
const math = std.math;

const bldc = @import("bldc.zig");
const csdk = bldc.csdk;
const stdio = bldc.stdio;
const pwm = bldc.pwm;
const foc = bldc.foc;

pub const Motor = struct {
    const Self = @This();

    u_axis_pins: pwm.PwmPair,
    v_axis_pins: pwm.PwmPair,
    w_axis_pins: pwm.PwmPair,

    pub fn create(u_axis_slice: pwm.Slice, v_axis_slice: pwm.Slice, w_axis_slice: pwm.Slice) Self {
        return Self{
            .u_axis_pins = pwm.PwmPair.create(u_axis_slice),
            .v_axis_pins = pwm.PwmPair.create(v_axis_slice),
            .w_axis_pins = pwm.PwmPair.create(w_axis_slice),
        };
    }

    pub fn init(self: Self) void {
        self.u_axis_pins.init();
        self.v_axis_pins.init();
        self.w_axis_pins.init();

        pwm.enableSlices(&[_]pwm.Slice{
            self.u_axis_pins.slice,
            self.v_axis_pins.slice,
            self.w_axis_pins.slice,
        });
    }

    pub fn setPwmFromVoltages(self: Self, voltages: foc.PhaseVoltage) void {
        // stdio.print("{}\n", .{voltages});
        self.u_axis_pins.setLevel(pwm.rescaleAsInt(u16, math.clamp(voltages.u_axis, -1, 1)));
        self.v_axis_pins.setLevel(pwm.rescaleAsInt(u16, math.clamp(voltages.v_axis, -1, 1)));
        self.w_axis_pins.setLevel(pwm.rescaleAsInt(u16, math.clamp(voltages.w_axis, -1, 1)));
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
    motor.setPwmFromVoltages(
        foc.getPhaseVoltage(1, 0, 0),
    );

    csdk.sleep_ms(500);

    const acceleration = 0.00000001;
    const max_speed = 0.05;
    var speed: f32 = 0.0;
    var angle: f32 = 0.0;

    while (true) {
        // stdio.print("angle:{}  speed:{}\n", .{ angle, speed });
        const phase_voltages = foc.getPhaseVoltage(1, 0, angle);
        motor.setPwmFromVoltages(phase_voltages);

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
