const bldc = @import("bldc.zig");
const stdio = bldc.stdio;
const csdk = bldc.csdk;

pub const PwmPair = struct {
    const Self = @This();

    slice: c_uint,

    pub fn create(slice: u8) !Self {
        const self = Self{
            .slice = slice,
        };

        return self;
    }

    pub fn initialize(self: Self) void {
        var config = csdk.pwm_get_default_config();

        csdk.pwm_config_set_output_polarity(&config, true, false);
        csdk.pwm_config_set_clkdiv_int(&config, 1);
        csdk.pwm_config_set_phase_correct(&config, true);

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

const PwmSlices = struct {
    const slice_u: u8 = 4;
    const slice_v: u8 = 6;
    const slice_w: u8 = 7;
};

pub fn demo() noreturn {
    const pwm_u = PwmPair.create(PwmSlices.slice_u) catch |err| {
        stdio.print("Error {}\n", .{err});
        @panic("dead");
    };
    const pwm_v = PwmPair.create(PwmSlices.slice_v) catch |err| {
        stdio.print("Error {}\n", .{err});
        @panic("dead");
    };
    const pwm_w = PwmPair.create(PwmSlices.slice_w) catch |err| {
        stdio.print("Error {}\n", .{err});
        @panic("dead");
    };

    csdk.gpio_set_function(8, csdk.GPIO_FUNC_PWM);
    csdk.gpio_set_function(9, csdk.GPIO_FUNC_PWM);
    csdk.gpio_set_function(12, csdk.GPIO_FUNC_PWM);
    csdk.gpio_set_function(13, csdk.GPIO_FUNC_PWM);
    csdk.gpio_set_function(14, csdk.GPIO_FUNC_PWM);
    csdk.gpio_set_function(15, csdk.GPIO_FUNC_PWM);

    pwm_u.initialize();
    pwm_v.initialize();
    pwm_w.initialize();

    // csdk.pwm_set_output_polarity(pwm_v.slice, false, true);

    // pwm_u.setLevel(0xffff / 2);
    // pwm_v.setLevel(0xffff / 2);
    pwm_w.setLevel(0);
    // csdk.pwm_set_output_polarity(PwmSlices.slice_w, false, false);

    //Enable all pwm signals at once
    csdk.pwm_set_mask_enabled((1 << PwmSlices.slice_u) | (1 << PwmSlices.slice_v) | (1 << PwmSlices.slice_w));

    var dir = enum { forwards, backwards }.forwards;

    var level: u16 = 0;
    const step = 5;

    while (true) {
        pwm_u.setLevel(level);
        pwm_v.setLevel(0xffff - level);

        switch (dir) {
            .forwards => {
                level += step;

                if (level == 0xffff) {
                    dir = .backwards;
                }
            },
            .backwards => {
                level -= step;

                if (level == 0x0) {
                    dir = .forwards;
                }
            },
        }

        csdk.sleep_us(200);
    }
}
