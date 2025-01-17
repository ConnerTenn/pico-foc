const sdk = @import("sdk-wrapper.zig");
pub const csdk = sdk.csdk;

pub const stdio = @import("stdio.zig");
pub const motor = @import("motor.zig");
pub const pwm = @import("pwm.zig");
pub const foc = @import("foc.zig");
pub const sensor = @import("sensor.zig");
pub const spi = @import("spi.zig");

pub const GPIO_IN = false;
pub const GPIO_OUT = true;

pub const GPIO_HIGH = true;
pub const GPIO_LOW = false;

pub const LED_PIN = csdk.PICO_DEFAULT_LED_PIN;

pub fn printBarGraph(size: comptime_int, value: f32, writer: anytype) !void {
    try writer.print("[", .{});
    for (0..size * 2 + 1) |i| {
        const idx = @as(i32, @intCast(i)) - size;
        const compare_val = @as(f32, @floatFromInt(idx)) / @as(f32, @floatFromInt(size));
        if (idx > 0) {
            try writer.print("{s}", .{if (compare_val <= value) "=" else " "});
        } else if (idx < 0) {
            try writer.print("{s}", .{if (compare_val >= value) "=" else " "});
        } else {
            try writer.print("|", .{});
        }
    }
    try writer.print("]", .{});
}
