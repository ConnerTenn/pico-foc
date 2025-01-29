const std = @import("std");
const math = std.math;

pub const motor = @import("motor.zig");
pub const pwm = @import("pwm.zig");
pub const foc = @import("foc.zig");
pub const sensor = @import("sensor.zig");
pub const spi = @import("spi.zig");
pub const duty_cycle = @import("duty_cycle.zig");

pub const ModType = enum {
    regular,
    mirror_y_axis,
    mirror_xy_axis,
};

pub inline fn mod(T: type, numerator: T, denominator: T, comptime mod_type: ModType) T {
    switch (mod_type) {
        .regular => {
            return math.mod(T, numerator, denominator) catch 0;
        },
        .mirror_y_axis => {
            const modulo = mod(T, numerator, denominator, .regular);

            //Check the sign of the original result
            if (numerator * denominator >= 0) {
                return modulo;
            } else {
                return denominator - modulo;
            }
        },
        .mirror_xy_axis => {
            const modulo = mod(T, numerator, denominator, .regular);

            //Check the sign of the original result
            if (numerator * denominator >= 0) {
                return modulo;
            } else {
                return modulo - denominator;
            }
        },
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
