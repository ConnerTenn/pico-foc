const bldc = @import("bldc.zig");
const csdk = bldc.csdk;
const stdio = bldc.stdio;

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
    runTristate();
}
