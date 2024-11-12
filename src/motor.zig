const std = @import("std");
const math = std.math;

const bldc = @import("bldc.zig");
const csdk = bldc.csdk;
const stdio = bldc.stdio;
const pwm = bldc.pwm;
const foc = bldc.foc;

const DriverPins = enum(c_uint) {
    UL = 8,
    UH = 9,
    VL = 12,
    VH = 13,
    WL = 14,
    WH = 15,

    fn init() void {
        const info = @typeInfo(DriverPins);
        inline for (info.Enum.fields) |field| {
            const pin = field.value;
            csdk.gpio_init(pin);
            csdk.gpio_set_dir(pin, bldc.GPIO_OUT);
        }
    }
};

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

const Tristate = enum {
    low,
    high,
    high_z,
};

const PinPair = struct {
    const Self = @This();

    h_pin: DriverPins,
    l_pin: DriverPins,

    inline fn setTristate(self: Self, tristate: Tristate) void {
        csdk.gpio_put(@intFromEnum(self.h_pin), switch (tristate) {
            .low => bldc.GPIO_LOW,
            .high => bldc.GPIO_HIGH,
            .high_z => bldc.GPIO_LOW,
        });
        csdk.gpio_put(@intFromEnum(self.l_pin), switch (tristate) {
            .low => bldc.GPIO_HIGH,
            .high => bldc.GPIO_LOW,
            .high_z => bldc.GPIO_LOW,
        });
    }
};

const TristatePhase = struct {
    const u_pins = PinPair{ .h_pin = DriverPins.UH, .l_pin = DriverPins.UL };
    const v_pins = PinPair{ .h_pin = DriverPins.VH, .l_pin = DriverPins.VL };
    const w_pins = PinPair{ .h_pin = DriverPins.WH, .l_pin = DriverPins.WL };

    u: Tristate,
    v: Tristate,
    w: Tristate,
};

const trapeziod_sequence = [_]TristatePhase{
    TristatePhase{ .u = .high, .v = .low, .w = .high_z },
    TristatePhase{ .u = .high, .v = .high_z, .w = .low },
    TristatePhase{ .u = .high_z, .v = .high, .w = .low },
    TristatePhase{ .u = .low, .v = .high, .w = .high_z },
    TristatePhase{ .u = .low, .v = .high_z, .w = .high },
    TristatePhase{ .u = .high_z, .v = .low, .w = .high },
};

pub fn runTristate() noreturn {
    DriverPins.init();

    var state_idx: u8 = 0;

    var delay: u64 = 10 * 1000;

    while (true) {
        const phase = trapeziod_sequence[state_idx];
        // stdio.print("{d}: {}\n", .{ state_idx, phase });
        TristatePhase.u_pins.setTristate(phase.u);
        TristatePhase.v_pins.setTristate(phase.v);
        TristatePhase.w_pins.setTristate(phase.w);

        state_idx += 1;
        if (state_idx >= trapeziod_sequence.len) {
            state_idx = 0;
        }

        // csdk.gpio_put(bldc.LED_PIN, led_toggle);
        // led_toggle = !led_toggle;

        csdk.sleep_us(delay);
        if (delay > 1500) {
            delay -= 5;
        }
    }
}

pub fn run() noreturn {
    // runTristate();

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
