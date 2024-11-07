const bldc = @import("bldc.zig");
const csdk = bldc.csdk;
const stdio = bldc.stdio;

const expect = @import("std").testing.expect;

export fn main() void {
    //Init prints
    _ = csdk.stdio_init_all();

    csdk.sleep_ms(1000);
    stdio.print("== BLDC Demo ==\n", .{});

    //Init GPIO
    csdk.gpio_init(bldc.LED_PIN);
    csdk.gpio_set_dir(bldc.LED_PIN, bldc.GPIO_OUT);

    bldc.motor.run();
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

test "trivial" {
    try expect(1 == 1);
}
