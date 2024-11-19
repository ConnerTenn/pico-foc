const std = @import("std");
const math = std.math;

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

pub inline fn mod(T: type, numerator: T, denominator: T) T {
    return math.mod(T, numerator, denominator) catch 0;
}

pub inline fn symetricMod(T: type, numerator: T, denominator: T) T {
    const modulo = mod(T, numerator, denominator);

    //Check the sign of the original result
    if (numerator * denominator >= 0) {
        return modulo;
    } else {
        return denominator - modulo;
    }
}

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

pub fn printPositionGraph(size: comptime_int, value: f32, lower: f32, upper: f32, writer: anytype) !void {
    try writer.print("[", .{});
    for (0..size) |i| {
        const compare_val = @as(f32, @floatFromInt(i)) / @as(f32, @floatFromInt(size));
        const step_size: f32 = 1.0 / @as(f32, @floatFromInt(size));

        //Convert to domain [0,1]
        const normalized_value = (value - lower) / (upper - lower);

        if (@abs(normalized_value - compare_val) < step_size) {
            try writer.print("|", .{});
        } else {
            try writer.print(" ", .{});
        }
    }
    try writer.print("]", .{});
}
