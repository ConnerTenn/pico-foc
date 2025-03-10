const std = @import("std");
const math = std.math;
const tau = math.tau;

const pico = @import("pico");
const csdk = pico.csdk;
const stdio = pico.stdio;
const hardware = pico.hardware;

const bldc = @import("bldc.zig");

const expect = @import("std").testing.expect;

fn printFrequencies() void {
    const f_pll_sys = csdk.frequency_count_khz(csdk.CLOCKS_FC0_SRC_VALUE_PLL_SYS_CLKSRC_PRIMARY);
    const f_pll_usb = csdk.frequency_count_khz(csdk.CLOCKS_FC0_SRC_VALUE_PLL_USB_CLKSRC_PRIMARY);
    const f_rosc = csdk.frequency_count_khz(csdk.CLOCKS_FC0_SRC_VALUE_ROSC_CLKSRC);
    const f_clk_sys = csdk.frequency_count_khz(csdk.CLOCKS_FC0_SRC_VALUE_CLK_SYS);
    const f_clk_peri = csdk.frequency_count_khz(csdk.CLOCKS_FC0_SRC_VALUE_CLK_PERI);
    const f_clk_usb = csdk.frequency_count_khz(csdk.CLOCKS_FC0_SRC_VALUE_CLK_USB);
    const f_clk_adc = csdk.frequency_count_khz(csdk.CLOCKS_FC0_SRC_VALUE_CLK_ADC);

    stdio.print("pll_sys  = {d}kHz\n", .{f_pll_sys});
    stdio.print("pll_usb  = {d}kHz\n", .{f_pll_usb});
    stdio.print("rosc     = {d}kHz\n", .{f_rosc});
    stdio.print("clk_sys  = {d}kHz\n", .{f_clk_sys});
    stdio.print("clk_peri = {d}kHz\n", .{f_clk_peri});
    stdio.print("clk_usb  = {d}kHz\n", .{f_clk_usb});
    stdio.print("clk_adc  = {d}kHz\n", .{f_clk_adc});
}

export fn main() void {
    //Init prints
    _ = csdk.stdio_init_all();

    csdk.sleep_ms(2000);
    stdio.print("== BLDC Demo ==\n", .{});
    printFrequencies();

    //Init GPIO
    hardware.gpio.default_led.init(hardware.gpio.Gpio.Config{
        .direction = .out,
    });

    for (0..31) |gpio| {
        stdio.print("io [{}] pwm slice:{} pwm chan:{}\n", .{
            gpio,
            csdk.pwm_gpio_to_slice_num(gpio),
            csdk.pwm_gpio_to_channel(gpio),
        });
    }

    csdk.gpio_set_function(8, csdk.GPIO_FUNC_PWM); //UL
    csdk.gpio_set_function(9, csdk.GPIO_FUNC_PWM); //UH
    csdk.gpio_set_function(12, csdk.GPIO_FUNC_PWM); //VL
    csdk.gpio_set_function(13, csdk.GPIO_FUNC_PWM); //VH
    csdk.gpio_set_function(14, csdk.GPIO_FUNC_PWM); //WL
    csdk.gpio_set_function(15, csdk.GPIO_FUNC_PWM); //WH

    var duty_cycle_sampler = pico.library.duty_cycle.DutyCycle.create(hardware.gpio.Pin.create(19), @as(hardware.gpio.Pin.Count, 1)) catch |err| {
        stdio.print("Error:{}\n", .{err});
        return;
    };
    duty_cycle_sampler.init();

    // while (true) {
    //     _ = duty_cycle_sampler.readDutyCycle();
    //     // stdio.print("Sample: {d:.3}\n", .{duty_cycle_sampler.readDutyCycle()});
    // }

    // const sensor = bldc.sensor.LIS3MDL.create(18, 19, 16, 17, csdk.spi0_hw);
    const sensor = duty_cycle_sampler.getSensor();

    const pid = pico.library.pid.PIDcontrol.create(
        3.0,
        0.0,
        0.05,
    );
    var motor = pico.library.motor.Motor.create(
        4,
        6,
        7,
        7,
        sensor,
        pid,
    );
    motor.init();
    // angleSweep(&motor);

    // // bldc.pwm.demo();
    // // bldc.sensor.demo();
    // // bldc.motor.run();
    angleTargetDemo(&motor);

    unreachable;
}

fn angleSweep(motor: *bldc.motor.Motor) void {
    var target_angle: f32 = 0.0;
    const start_time_us = csdk.get_absolute_time();
    const rot_per_sec = 0.1;

    while (target_angle < math.tau) {
        //Drive to the target angle
        motor.setTorque(1.0, 0.0, target_angle);
        stdio.print("\n", .{});

        //Update target angle
        const current_time_us = csdk.get_absolute_time();
        const us_per_sec = 1_000_000.0;

        target_angle = rot_per_sec * tau * @as(f32, @floatFromInt(current_time_us - start_time_us)) / us_per_sec;
    }
}

fn targetAngle(angle: f32, delta_time_s: f32) f32 {
    _ = delta_time_s; // autofix
    const delta_err = pico.math.deltaError(f32, angle, 0, tau / 64.0);
    // stdio.print("{d:.3}\n", .{delta_err});

    const dead_zone = 0.01;

    if (@abs(delta_err) < dead_zone) {
        return 0;
    }
    return delta_err * 8.0;
}

fn angleTargetDemo(motor: *pico.library.motor.Motor) noreturn {
    motor.target.velocity = math.tau * 1;
    motor.target.torque = 1;
    while (true) {
        motor.update(targetAngle);
    }
}

test "trivial" {
    try expect(1 == 1);
}
