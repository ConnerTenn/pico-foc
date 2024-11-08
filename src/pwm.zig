const bldc = @import("bldc.zig");
const stdio = bldc.stdio;
const csdk = bldc.csdk;

pub const Pwm = struct {
    const Self = @This();

    slice: c_uint,
    channel: c_uint,

    pub fn create(pin: u32) Self {
        csdk.gpio_set_function(pin, csdk.GPIO_FUNC_PWM);

        return Self{
            .slice = csdk.pwm_gpio_to_slice_num(pin),
            .channel = csdk.pwm_gpio_to_channel(pin),
        };
    }

    pub inline fn setLevel(self: Self, level: u16) void {
        csdk.pwm_set_chan_level(self.slice, self.channel, level);
    }
};

pub const PwmPair = struct {
    const Self = @This();

    pwm_a: Pwm,
    pwm_b: Pwm,
    config: csdk.pwm_config,

    pub fn create(pin_a: u32, pin_b: u32) !Self {
        var config = csdk.pwm_get_default_config();

        csdk.pwm_config_set_output_polarity(&config, false, true);
        csdk.pwm_config_set_clkdiv_int(&config, 255);

        const self = Self{
            .pwm_a = Pwm.create(pin_a),
            .pwm_b = Pwm.create(pin_b),
            .config = config,
        };

        if (self.pwm_a.slice != self.pwm_b.slice or self.pwm_a.channel == self.pwm_b.channel) {
            return error.incompatible_pins;
        }

        return self;
    }

    pub fn enable(self: *Self) void {
        csdk.pwm_init(self.pwm_a.slice, &self.config, true);
    }

    pub fn disable(self: Self) void {
        csdk.pwm_set_enabled(self.pwm_a.slice, false);
    }

    pub inline fn setLevel(self: Self, level: u16) void {
        csdk.pwm_set_both_levels(self.pwm_a.slice, level, level);
    }
};

pub fn demo() noreturn {
    var pwm_u = PwmPair.create(8, 9) catch |err| {
        stdio.print("Error {}", .{err});
        @panic("dead");
    };
    // const pwm_v = PwmPair.create(12, 13) catch |err| {
    //     stdio.print("Error {}", .{err});
    // };
    // _ = pwm_v; // autofix
    // const pwm_w = PwmPair.create(14, 15) catch |err| {
    //     stdio.print("Error {}", .{err});
    // };
    // _ = pwm_w; // autofix

    pwm_u.enable();
    pwm_u.setLevel(0xffff / 2);

    while (true) {}
}
