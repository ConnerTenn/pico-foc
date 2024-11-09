const sdk = @import("sdk-wrapper.zig");
pub const csdk = sdk.csdk;

pub const stdio = @import("stdio.zig");
pub const motor = @import("motor.zig");
pub const pwm = @import("pwm.zig");
pub const foc = @import("foc.zig");

pub const GPIO_IN = false;
pub const GPIO_OUT = true;

pub const GPIO_HIGH = true;
pub const GPIO_LOW = false;

pub const LED_PIN = csdk.PICO_DEFAULT_LED_PIN;
