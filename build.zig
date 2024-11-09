const std = @import("std");
const Build = std.Build;

pub fn build(b: *Build) void {
    const target = std.Target.Query{
        .os_tag = .freestanding,
        .cpu_arch = .thumb,
        .cpu_model = .{
            .explicit = &std.Target.arm.cpu.cortex_m0plus,
        },
        .abi = .eabi,
    };

    const options = Build.StaticLibraryOptions{
        .name = "bldc",
        .optimize = b.standardOptimizeOption(Build.StandardOptimizeOptionOptions{
            .preferred_optimize_mode = .ReleaseFast,
        }),
        .target = b.standardTargetOptions(Build.StandardTargetOptionsArgs{
            .default_target = target,
        }),
        .root_source_file = b.path("src/main.zig"),
    };
    // Standard release options allow the person running `zig build` to select
    // between Debug, ReleaseSafe, ReleaseFast, and ReleaseSmall.
    // const mode = b.standardReleaseOptions();

    const lib = b.addStaticLibrary(options);
    // lib.linkLibC();
    // lib.linkSystemLibrary("c");

    const includes = [_][]const u8{
        // "./gen",
        // "/usr/lib/gcc/arm-none-eabi/12.1.0/../../../../arm-none-eabi/include/",

        // "./pico-sdk/bazel/include",
        "./pico-sdk/lib/btstack/3rd-party/bluedroid/decoder/include",
        "./pico-sdk/lib/btstack/3rd-party/bluedroid/encoder/include",
        "./pico-sdk/lib/btstack/3rd-party/lc3-google/include",
        "./pico-sdk/lib/btstack/3rd-party/lwip/core/src/include",
        "./pico-sdk/lib/btstack/port/esp32/components/btstack/include",
        "./pico-sdk/lib/btstack/port/samv71-xplained-atwilc3000/ASF/sam/utils/cmsis/samv71/include",
        "./pico-sdk/lib/btstack/test/le_audio/include",
        "./pico-sdk/lib/lwip/contrib/ports/freertos/include",
        "./pico-sdk/lib/lwip/contrib/ports/unix/port/include",
        "./pico-sdk/lib/lwip/contrib/ports/unix/posixlib/include",
        "./pico-sdk/lib/lwip/contrib/ports/win32/include",
        "./pico-sdk/lib/lwip/src/include",
        "./pico-sdk/lib/mbedtls/3rdparty/everest/include",
        "./pico-sdk/lib/mbedtls/include",
        "./pico-sdk/lib/mbedtls/tests/include",
        "./pico-sdk/lib/tinyusb/hw/bsp/espressif/components/led_strip/include",
        "./pico-sdk/lib/tinyusb/hw/bsp/fomu/include",
        "./pico-sdk/lib/tinyusb/hw/mcu/dialog/da1469x/SDK_10.0.8.105/sdk/bsp/include",
        "./pico-sdk/lib/tinyusb/hw/mcu/dialog/da1469x/include",
        "./pico-sdk/lib/tinyusb/hw/mcu/nordic/nrf5x/s140_nrf52_6.1.1_API/include",
        "./pico-sdk/src/boards/include",
        "./pico-sdk/src/common/boot_picobin_headers/include",
        "./pico-sdk/src/common/boot_picoboot_headers/include",
        "./pico-sdk/src/common/boot_uf2_headers/include",
        "./pico-sdk/src/common/hardware_claim/include",
        "./pico-sdk/src/common/pico_base_headers/include",
        "./pico-sdk/src/common/pico_binary_info/include",
        "./pico-sdk/src/common/pico_bit_ops_headers/include",
        "./pico-sdk/src/common/pico_divider_headers/include",
        "./pico-sdk/src/common/pico_stdlib_headers/include",
        "./pico-sdk/src/common/pico_sync/include",
        "./pico-sdk/src/common/pico_time/include",
        "./pico-sdk/src/common/pico_usb_reset_interface_headers/include",
        "./pico-sdk/src/common/pico_util/include",
        "./pico-sdk/src/rp2040/boot_stage2/include",
        "./pico-sdk/src/rp2040/hardware_regs/include",
        "./pico-sdk/src/rp2040/hardware_structs/include",
        "./pico-sdk/src/rp2040/pico_platform/include",
        // "./pico-sdk/src/rp2350/boot_stage2/include",
        // "./pico-sdk/src/rp2350/hardware_regs/include",
        // "./pico-sdk/src/rp2350/hardware_structs/include",
        // "./pico-sdk/src/rp2350/pico_platform/include",
        "./pico-sdk/src/rp2_common/cmsis/include",
        "./pico-sdk/src/rp2_common/hardware_adc/include",
        "./pico-sdk/src/rp2_common/hardware_base/include",
        "./pico-sdk/src/rp2_common/hardware_boot_lock/include",
        "./pico-sdk/src/rp2_common/hardware_clocks/include",
        "./pico-sdk/src/rp2_common/hardware_dcp/include",
        "./pico-sdk/src/rp2_common/hardware_divider/include",
        "./pico-sdk/src/rp2_common/hardware_dma/include",
        "./pico-sdk/src/rp2_common/hardware_exception/include",
        "./pico-sdk/src/rp2_common/hardware_flash/include",
        "./pico-sdk/src/rp2_common/hardware_gpio/include",
        "./pico-sdk/src/rp2_common/hardware_hazard3/include",
        "./pico-sdk/src/rp2_common/hardware_i2c/include",
        "./pico-sdk/src/rp2_common/hardware_interp/include",
        "./pico-sdk/src/rp2_common/hardware_irq/include",
        "./pico-sdk/src/rp2_common/hardware_pio/include",
        "./pico-sdk/src/rp2_common/hardware_pll/include",
        "./pico-sdk/src/rp2_common/hardware_powman/include",
        "./pico-sdk/src/rp2_common/hardware_pwm/include",
        "./pico-sdk/src/rp2_common/hardware_rcp/include",
        "./pico-sdk/src/rp2_common/hardware_resets/include",
        "./pico-sdk/src/rp2_common/hardware_riscv/include",
        "./pico-sdk/src/rp2_common/hardware_riscv_platform_timer/include",
        "./pico-sdk/src/rp2_common/hardware_rtc/include",
        "./pico-sdk/src/rp2_common/hardware_sha256/include",
        "./pico-sdk/src/rp2_common/hardware_spi/include",
        "./pico-sdk/src/rp2_common/hardware_sync/include",
        "./pico-sdk/src/rp2_common/hardware_sync_spin_lock/include",
        "./pico-sdk/src/rp2_common/hardware_ticks/include",
        "./pico-sdk/src/rp2_common/hardware_timer/include",
        "./pico-sdk/src/rp2_common/hardware_uart/include",
        "./pico-sdk/src/rp2_common/hardware_vreg/include",
        "./pico-sdk/src/rp2_common/hardware_watchdog/include",
        "./pico-sdk/src/rp2_common/hardware_xosc/include",
        "./pico-sdk/src/rp2_common/pico_aon_timer/include",
        "./pico-sdk/src/rp2_common/pico_async_context/include",
        "./pico-sdk/src/rp2_common/pico_atomic/include",
        "./pico-sdk/src/rp2_common/pico_bootrom/include",
        "./pico-sdk/src/rp2_common/pico_btstack/include",
        "./pico-sdk/src/rp2_common/pico_clib_interface/include",
        "./pico-sdk/src/rp2_common/pico_cyw43_arch/include",
        "./pico-sdk/src/rp2_common/pico_cyw43_driver/include",
        "./pico-sdk/src/rp2_common/pico_double/include",
        "./pico-sdk/src/rp2_common/pico_fix/rp2040_usb_device_enumeration/include",
        "./pico-sdk/src/rp2_common/pico_flash/include",
        "./pico-sdk/src/rp2_common/pico_float/include",
        "./pico-sdk/src/rp2_common/pico_i2c_slave/include",
        "./pico-sdk/src/rp2_common/pico_int64_ops/include",
        "./pico-sdk/src/rp2_common/pico_lwip/include",
        "./pico-sdk/src/rp2_common/pico_malloc/include",
        "./pico-sdk/src/rp2_common/pico_mbedtls/include",
        "./pico-sdk/src/rp2_common/pico_mem_ops/include",
        "./pico-sdk/src/rp2_common/pico_multicore/include",
        "./pico-sdk/src/rp2_common/pico_platform_compiler/include",
        "./pico-sdk/src/rp2_common/pico_platform_panic/include",
        "./pico-sdk/src/rp2_common/pico_platform_sections/include",
        "./pico-sdk/src/rp2_common/pico_printf/include",
        "./pico-sdk/src/rp2_common/pico_rand/include",
        "./pico-sdk/src/rp2_common/pico_runtime/include",
        "./pico-sdk/src/rp2_common/pico_runtime_init/include",
        "./pico-sdk/src/rp2_common/pico_sha256/include",
        "./pico-sdk/src/rp2_common/pico_stdio/include",
        "./pico-sdk/src/rp2_common/pico_stdio_rtt/include",
        "./pico-sdk/src/rp2_common/pico_stdio_semihosting/include",
        "./pico-sdk/src/rp2_common/pico_stdio_uart/include",
        "./pico-sdk/src/rp2_common/pico_stdio_usb/include",
        "./pico-sdk/src/rp2_common/pico_time_adapter/include",
        "./pico-sdk/src/rp2_common/pico_unique_id/include",
        "./pico-sdk/src/rp2_common/tinyusb/include",
        // "./pico-sdk/test/pico_test/include",
        // "./build/_deps/picotool-src/lib/include",
        // "./build/_deps/picotool-build/lib/mbedtls/include",

        "./build/generated/pico_base",
    };

    inline for (includes) |include| {
        lib.addIncludePath(b.path(include));
    }

    const arm_includes = [_][]const u8{
        "/nix/store/3w7l1k6ip6x0yrl7pfqx7mhpr0j0mrrs-gcc-arm-embedded-13.2.rel1/bin/../lib/gcc/arm-none-eabi/13.2.1/include",
        "/nix/store/3w7l1k6ip6x0yrl7pfqx7mhpr0j0mrrs-gcc-arm-embedded-13.2.rel1/bin/../lib/gcc/arm-none-eabi/13.2.1/include-fixed",
        "/nix/store/3w7l1k6ip6x0yrl7pfqx7mhpr0j0mrrs-gcc-arm-embedded-13.2.rel1/bin/../lib/gcc/arm-none-eabi/13.2.1/../../../../arm-none-eabi/include",
    };

    inline for (arm_includes) |include| {
        lib.addIncludePath(.{
            .cwd_relative = include,
        });
    }

    const lib_artifact = b.addInstallArtifact(lib, .{});

    const build_step = b.step("build", "Build the application static library");
    build_step.dependOn(&lib_artifact.step);

    // //Tests
    // const tests = b.addTest(Build.TestOptions{
    //     .name = "tests",
    //     .target = defaultTarget(),
    //     .root_source_file = b.path("src/main.zig"),
    // });
    // // tests.linkLibC();
    // // tests.linkSystemLibrary("c");

    // const tests_step = b.step("test", "Run the tests");
    // tests_step.dependOn(&tests.step);
}

fn defaultTarget() Build.ResolvedTarget {
    const query = std.Target.Query{};
    return Build.ResolvedTarget{
        .query = query,
        .result = std.zig.system.resolveTargetQuery(query) catch
            @panic("unable to resolve target query"),
    };
}
