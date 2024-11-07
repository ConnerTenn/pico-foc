const std = @import("std");
const bldc = @import("bldc.zig");
const csdk = bldc.csdk;

var print_buffer: [1024]u8 = .{0} ** 1024;

fn writeFn(context: *const anyopaque, bytes: []const u8) anyerror!usize {
    _ = context; // autofix
    const len = csdk.stdio_put_string(bytes.ptr, @intCast(bytes.len), false, false);
    return @intCast(len);
}

pub fn print(comptime fmt: []const u8, args: anytype) void {
    std.fmt.format(
        std.io.AnyWriter{
            .context = @ptrCast(&writeFn),
            .writeFn = writeFn,
        },
        fmt,
        args,
    ) catch {};
}
