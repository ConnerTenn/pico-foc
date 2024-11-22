const std = @import("std");
const math = std.math;
const tau = math.tau;

const bldc = @import("bldc.zig");
const csdk = bldc.csdk;
const stdio = bldc.stdio;

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
    csdk.gpio_init(bldc.LED_PIN);
    csdk.gpio_set_dir(bldc.LED_PIN, bldc.GPIO_OUT);

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

    const sensor = bldc.sensor.LIS3MDL.create(18, 19, 16, 17, csdk.spi0_hw);
    var motor = bldc.motor.Motor.create(
        4,
        6,
        7,
        7,
        sensor,
    );
    motor.init();

    // bldc.pwm.demo();
    // bldc.sensor.demo();
    // bldc.motor.run();
    // angleTargetDemo(&motor);

    while (true) {
        motor.update();
    }

    // //Blink loop
    // while (true) {
    //     stdio.print("Pico!", .{});
    //     csdk.gpio_put(bldc.LED_PIN, bldc.GPIO_HIGH);
    //     csdk.sleep_ms(250);

    //     csdk.gpio_put(bldc.LED_PIN, bldc.GPIO_LOW);
    //     csdk.sleep_ms(250);
    // }

    unreachable;
}

// fn angleTargetDemo(motor: *bldc.motor.Motor) noreturn {

//     motor.target.velocity = math.tau * 1;
//     motor.target.torque = 1;
//     while (true) {
//         motor.update();
//     }
// }

test "trivial" {
    try expect(1 == 1);
}
