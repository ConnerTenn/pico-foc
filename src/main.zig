const expect = @import("std").testing.expect;

const sdk = @import("sdk-wrapper.zig");
const csdk = sdk.csdk;

export fn main() void {
    //Init prints
    _ = csdk.stdio_init_all();

    //Init GPIO
    csdk.gpio_init(sdk.LED_PIN);
    csdk.gpio_set_dir(sdk.LED_PIN, sdk.GPIO_OUT);

    //Blink loop
    while (true) {
        _ = csdk.printf("Pico!");
        csdk.gpio_put(sdk.LED_PIN, sdk.GPIO_HIGH);
        csdk.sleep_ms(250);

        csdk.gpio_put(sdk.LED_PIN, sdk.GPIO_LOW);
        csdk.sleep_ms(250);
    }

    unreachable;
}

test "trivial" {
    try expect(1 == 1);
}
