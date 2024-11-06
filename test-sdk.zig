//== DUMMY SDK ==

const std = @import("std");

pub const PICO_DEFAULT_LED_PIN = 25;

pub fn stdio_init_all() void {}

pub fn gpio_init(_: i32) void {}
pub fn gpio_set_dir(_: i32, _: bool) void {}
pub fn gpio_put(_: i32, _: bool) void {}

pub fn sleep_ms(_: i32) void {}

// const c = @cImport({
//     @cInclude("stdio.h");
// });

// pub const printf = c.printf;
pub fn printf(fmt: [*:0]const u8, ...) callconv(.C) void {
    _ = fmt; // autofix
}
